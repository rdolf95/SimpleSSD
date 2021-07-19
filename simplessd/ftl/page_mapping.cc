/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ftl/page_mapping.hh"

#include <algorithm>
#include <limits>
#include <random>

#include "util/algorithm.hh"
#include "util/bitset.hh"

namespace SimpleSSD {

namespace FTL {

PageMapping::PageMapping(ConfigReader &c, Parameter &p, PAL::PAL *l,
                         DRAM::AbstractDRAM *d)
    : AbstractFTL(p, l, d),
      pPAL(l),
      conf(c),
      lastFreeBlock(param.pageCountToMaxPerf),
      lastFreeBlockIOMap(param.ioUnitInPage),
      bReclaimMore(false),
      errorCountStat(128),
      lastFreeBlock2(2, vector<uint32_t>(param.pageCountToMaxPerf)),
      lastFreeBlockIOMap2(2, Bitset(param.ioUnitInPage)),
      lastFreeBlockIndex2(2),
      lruAverage(0){
  blocks.reserve(param.totalPhysicalBlocks);
  table.reserve(param.totalLogicalBlocks * param.pagesInBlock);

  for (uint32_t i = 0; i < param.totalPhysicalBlocks; i++) {
    freeBlocks.emplace_back(Block(i, param.pagesInBlock, param.ioUnitInPage));
  }

  // Push free blocks to free block list of error count = 0
  uint32_t eccCapability = conf.readInt(CONFIG_FTL, FTL_ECC_CAPABILITY);

  for (uint32_t i = 0; i < eccCapability; i++){
    std::list<Block> freeList;
    std::list<uint32_t> validList;
    errorCountTable.push_back(make_pair(freeList, validList));
  }


  // BER modeling data  
  static uint64_t eraseThreshold =
      conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  static float initialBER =
      conf.readFloat(CONFIG_FTL, FTL_INITIAL_BER);
  static float finalBER =
      conf.readFloat(CONFIG_FTL, FTL_FINAL_BER);
  static float sigma =
      conf.readFloat(CONFIG_FTL, FTL_BER_SIGMA);
  static uint64_t initErase =
    conf.readInt(CONFIG_FTL, FTL_INITIAL_ERASE);

  // BER modeling random generator
  std::random_device generator;
  uint32_t errorCount;
  float averageError;
  float temp;

  sigma = 2;


  for (uint32_t i = 0; i < param.totalPhysicalBlocks; i++) {
    averageError = (finalBER - initialBER)/eraseThreshold
                      * initErase + initialBER;
    averageError = averageError * param.pageSize * 8;
    std::normal_distribution<double> normal(averageError,sigma);

    // Remove negative value
    temp = normal(generator);
    if (temp < 0){
      temp = 0;
    }

    // Keep maximum error count ever seen
    errorCount = MAX(static_cast<uint32_t>(temp + 0.5), 0);
    if(errorCount>=128){
      errorCount = 127;
    }
    
    errorCountTable[errorCount].first.emplace_back(Block(i, param.pagesInBlock, param.ioUnitInPage));
    errorCountTable[errorCount].first.back().updateError(errorCount);
    errorCountTable[errorCount].first.back().updateEraseCount(initErase);
    errorCountStat[errorCount] ++;
  }

  lruWindowSize = conf.readInt(CONFIG_FTL, FTL_LRU_MAX);
  vulBlockCount = 0;
  badBlockCount = 0;

  nFreeBlocks = param.totalPhysicalBlocks;

  status.totalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;

  // Allocate free blocks
  //for (uint32_t i = 0; i < param.pageCountToMaxPerf; i++) {
  //  lastFreeBlock.at(i) = getFreeBlock(i);
  //}

  //for (uint32_t i = 0; i < param.pageCountToMaxPerf; i++) {
  //  lastFreeBlock.at(i) = getFreeBlock2(0, 1);
  //}

  //lastFreeBlockIndex = 0;

  for (uint32_t j = 0; j < 2; j++){
    for (uint32_t i = 0; i < param.pageCountToMaxPerf; i++) {
      lastFreeBlock2[j].at(i) = getFreeBlock2(0, 1);
    }
    lastFreeBlockIndex2[j] = 0;
  }
  

  memset(&stat, 0, sizeof(stat));
  memset(&wearLevelingStat, 0, sizeof(wearLevelingStat));
  
  wearLevelingStat.minErrorCount = 0;
  wearLevelingStat.strongBoundary = 32;//32 //65
  wearLevelingStat.weakBoundary = 40;//40 //75



  bRandomTweak = conf.readBoolean(CONFIG_FTL, FTL_USE_RANDOM_IO_TWEAK);
  bitsetSize = bRandomTweak ? param.ioUnitInPage : 1;
}

PageMapping::~PageMapping() {}

bool PageMapping::initialize() {
  uint64_t nPagesToWarmup;
  uint64_t nPagesToInvalidate;
  uint64_t nTotalLogicalPages;
  uint64_t maxPagesBeforeGC;
  uint64_t tick;
  uint64_t valid;
  uint64_t invalid;
  FILLING_MODE mode;

  Request req(param.ioUnitInPage);

  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization started");

  nTotalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;
  nPagesToWarmup =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_FILL_RATIO);
  nPagesToInvalidate =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_INVALID_PAGE_RATIO);
  mode = (FILLING_MODE)conf.readUint(CONFIG_FTL, FTL_FILLING_MODE);
  maxPagesBeforeGC =
      param.pagesInBlock *
      (param.totalPhysicalBlocks *
           (1 - conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO)) -
       param.pageCountToMaxPerf);  // # free blocks to maintain

  if (nPagesToWarmup + nPagesToInvalidate > maxPagesBeforeGC) {
    warn("ftl: Too high filling ratio. Adjusting invalidPageRatio.");
    nPagesToInvalidate = maxPagesBeforeGC - nPagesToWarmup;
  }

  debugprint(LOG_FTL_PAGE_MAPPING, "Total logical pages: %" PRIu64,
             nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total logical pages to fill: %" PRIu64 " (%.2f %%)",
             nPagesToWarmup, nPagesToWarmup * 100.f / nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total invalidated pages to create: %" PRIu64 " (%.2f %%)",
             nPagesToInvalidate,
             nPagesToInvalidate * 100.f / nTotalLogicalPages);

  req.ioFlag.set();

  // Step 1. Filling
  if (mode == FILLING_MODE_0 || mode == FILLING_MODE_1) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal2(req, tick, false);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal2(req, tick, false);
    }
  }
  

  // Step 2. Invalidating
  if (mode == FILLING_MODE_0) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal2(req, tick, false);
    }
  }
  else if (mode == FILLING_MODE_1) {
    // Random
    // We can successfully restrict range of LPN to create exact number of
    // invalid pages because we wrote in sequential mannor in step 1.
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nPagesToWarmup - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal2(req, tick, false);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal2(req, tick, false);
    }
  }

  // Report
  calculateTotalPages(valid, invalid);
  debugprint(LOG_FTL_PAGE_MAPPING, "Filling finished. Page status:");
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total valid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             valid, valid * 100.f / nTotalLogicalPages, nPagesToWarmup,
             (int64_t)(valid - nPagesToWarmup));
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total invalid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             invalid, invalid * 100.f / nTotalLogicalPages, nPagesToInvalidate,
             (int64_t)(invalid - nPagesToInvalidate));
  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization finished");

  return true;
}

void PageMapping::read(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    readInternal2(req, tick);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "READ  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ);
}

void PageMapping::write(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    writeInternal2(req, tick);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE);
}

void PageMapping::trim(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  trimInternal(req, tick);

  debugprint(LOG_FTL_PAGE_MAPPING,
             "TRIM  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
             ")",
             req.lpn, begin, tick, tick - begin);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM);
}

void PageMapping::format(LPNRange &range, uint64_t &tick) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<uint32_t> list;

  req.ioFlag.set();

  for (auto iter = table.begin(); iter != table.end();) {
    if (iter->first >= range.slpn && iter->first < range.slpn + range.nlp) {
      auto &mappingList = iter->second;

      // Do trim
      for (uint32_t idx = 0; idx < bitsetSize; idx++) {
        auto &mapping = mappingList.at(idx);
        auto block = blocks.find(mapping.first);

        if (block == blocks.end()) {
          panic("Block is not in use");
        }

        block->second.invalidate(mapping.second, idx);

        // Collect block indices
        list.push_back(mapping.first);
      }

      iter = table.erase(iter);
    }
    else {
      iter++;
    }
  }

  // Get blocks to erase
  std::sort(list.begin(), list.end());
  auto last = std::unique(list.begin(), list.end());
  list.erase(last, list.end());

  // Do GC only in specified blocks
  doGarbageCollection(list, tick);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::FORMAT);
}

Status *PageMapping::getStatus(uint64_t lpnBegin, uint64_t lpnEnd) {
  status.freePhysicalBlocks = nFreeBlocks;

  if (lpnBegin == 0 && lpnEnd >= status.totalLogicalPages) {
    status.mappedLogicalPages = table.size();
  }
  else {
    status.mappedLogicalPages = 0;

    for (uint64_t lpn = lpnBegin; lpn < lpnEnd; lpn++) {
      if (table.count(lpn) > 0) {
        status.mappedLogicalPages++;
      }
    }
  }

  return &status;
}

float PageMapping::freeBlockRatio() {
  return (float)nFreeBlocks / param.totalPhysicalBlocks;
}

uint32_t PageMapping::convertBlockIdx(uint32_t blockIdx) {
  return blockIdx % param.pageCountToMaxPerf;
}

uint32_t PageMapping::getFreeBlock(uint32_t idx) {
  uint32_t blockIndex = 0;

  if (idx >= param.pageCountToMaxPerf) {
    panic("Index out of range");
  }

  if (nFreeBlocks > 0) {
    // Search block which is blockIdx % param.pageCountToMaxPerf == idx
    auto iter = freeBlocks.begin();

    for (; iter != freeBlocks.end(); iter++) {
      blockIndex = iter->getBlockIndex();

      if (blockIndex % param.pageCountToMaxPerf == idx) {
        break;
      }
    }

    // Sanity check
    if (iter == freeBlocks.end()) {
      // Just use first one
      iter = freeBlocks.begin();
      blockIndex = iter->getBlockIndex();
    }

    // Insert found block to block list
    if (blocks.find(blockIndex) != blocks.end()) {
      panic("Corrupted");
    }

    blocks.emplace(blockIndex, std::move(*iter));

    // Remove found block from free block list
    freeBlocks.erase(iter);
    nFreeBlocks--;
  }
  else {
    panic("No free block left");
  }

  return blockIndex;
}

uint32_t PageMapping::getLastFreeBlock(Bitset &iomap) {
  if (!bRandomTweak || (lastFreeBlockIOMap & iomap).any()) {
    // Update lastFreeBlockIndex
    lastFreeBlockIndex++;

    if (lastFreeBlockIndex == param.pageCountToMaxPerf) {
      lastFreeBlockIndex = 0;
    }

    lastFreeBlockIOMap = iomap;
  }
  else {
    lastFreeBlockIOMap |= iomap;
  }

  auto freeBlock = blocks.find(lastFreeBlock.at(lastFreeBlockIndex));

  // Sanity check
  if (freeBlock == blocks.end()) {
    panic("Corrupted");
  }

  // If current free block is full, get next block
  if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
    //lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock(lastFreeBlockIndex);
    lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock2(0, 1);
    bReclaimMore = true;
  }
  

  return lastFreeBlock.at(lastFreeBlockIndex);
}

/*
  errorIdx : index of errorCountTable 
             It should be 
             minErrorCount or,
             minErrorCount + strongSize or,
             minErrorCount + strongSize + weakSize
*/
uint32_t PageMapping::getFreeBlock2(uint16_t errorIdx, uint32_t isHot) {
  uint32_t blockIndex = 0;
  uint16_t minError = wearLevelingStat.minErrorCount;
  uint16_t strongBoundary = wearLevelingStat.strongBoundary;
  uint16_t weakBoundary = wearLevelingStat.weakBoundary;

  // TODO: For testing
  minError = 0;
  
  if (errorIdx != minError 
      && errorIdx != strongBoundary
      && errorIdx != weakBoundary) {
    panic("Unallowed free page request");
  }

  // Find free block with more than errorIdx
  //auto tableIter = errorCountTable.begin() + errorIdx;
  auto tableIter = errorCountTable.begin();
  if (isHot == 1) // Hot
  {  
    uint32_t iterCount = minError;
    tableIter = errorCountTable.begin();
    while (tableIter->first.empty() && (iterCount < strongBoundary))
    { // Strong blocks
      iterCount ++;
      tableIter = errorCountTable.begin() + iterCount;
    }
    
    while (tableIter->first.empty() && (iterCount < weakBoundary))
    { // Weak blocks
      iterCount ++;
      tableIter = errorCountTable.begin() + iterCount;
    }
    
    if(!(tableIter->first.empty())){
      wearLevelingStat.strongBoundary = iterCount;
    }
    
    while (tableIter->first.empty() && (tableIter != errorCountTable.end()))
    { // Vulnerable blocks
      tableIter ++;
    }
  }
  else if (isHot == 0)  //Cold
  {
    uint32_t iterCount = weakBoundary;
    tableIter = errorCountTable.begin() + weakBoundary;
    while (tableIter->first.empty() && (iterCount > strongBoundary))
    { // Weak blocks
      iterCount --;
      tableIter = errorCountTable.begin() + iterCount;
    }
    
    while (tableIter->first.empty() && (iterCount > minError))
    { // Strong blocks
      iterCount --;
      tableIter = errorCountTable.begin() + iterCount;
    }

    if(!(tableIter->first.empty())){
      wearLevelingStat.strongBoundary = iterCount;
    }
    
    while (tableIter->first.empty() && (tableIter != errorCountTable.end()))
    { // Vunerable blocks
      tableIter ++;
    }
  }

  // TODO: Must adjust boundaries of strong/weak if table Iter exceeded them.
  if (tableIter != errorCountTable.end())
  {
    // Use first one of minimum 
    blockIndex = tableIter->first.front().getBlockIndex();

    // Save error count table index to block meta data
    uint32_t errorCount = tableIter->first.front().getBitErrorCount();
    //debugprint(LOG_FTL_PAGE_MAPPING, "get free block: error count: %u", errorCount);
    tableIter->first.front().updateErrorTableIndex(errorCount);

    // Insert found block to valid block list
    auto newTableIter = errorCountTable.begin() + errorCount;
    newTableIter->second.emplace_back(blockIndex);
  
    if (blocks.find(blockIndex) != blocks.end()) {
      panic("Corrupted free block");
    }

    //blocks.emplace(blockIndex, Block(tableIter->first.front()));
    blocks.emplace(blockIndex, std::move(tableIter->first.front()));
    //blocks.emplace(blockIndex, tableIter->first.front());

    // Remove found block from free block list
    tableIter->first.pop_front();
    nFreeBlocks--;
  }
  else{
    panic("No free block left");
  }

  return blockIndex;
}

uint32_t PageMapping::getLastFreeBlock2(Bitset &iomap, uint32_t isHot) {
  //isHot == 0 : cold, isHot == 1 : hot
  if (!bRandomTweak || (lastFreeBlockIOMap2[isHot] & iomap).any()) {
    // Update lastFreeBlockIndex
    lastFreeBlockIndex2[isHot]++;

    if (lastFreeBlockIndex2[isHot] == param.pageCountToMaxPerf) {
      lastFreeBlockIndex2[isHot] = 0;
    }

    lastFreeBlockIOMap2[isHot] = iomap;
  }
  else {
    lastFreeBlockIOMap2[isHot] |= iomap;
  }
  
  auto freeBlock = blocks.find(lastFreeBlock2[isHot].at(lastFreeBlockIndex2[isHot]));

  // Sanity check
  // Because now we use two last free block lists, so freeBlock can be erased. (May be)
  // So, let's just get new free block instead.
  if (freeBlock == blocks.end()) {
    //panic("Corrupted last free block");
    lastFreeBlock2[isHot].at(lastFreeBlockIndex2[isHot]) = getFreeBlock2(0, isHot);
    freeBlock = blocks.find(lastFreeBlock2[isHot].at(lastFreeBlockIndex2[isHot]));
    bReclaimMore = true;
  }

  // If current free block is full, get next block
  if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
    //lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock(lastFreeBlockIndex);
    lastFreeBlock2[isHot].at(lastFreeBlockIndex2[isHot]) = getFreeBlock2(0, isHot);
    bReclaimMore = true;
  }
  

  return lastFreeBlock2[isHot].at(lastFreeBlockIndex2[isHot]);
}

// calculate weight of each block regarding victim selection policy
void PageMapping::calculateVictimWeight(
    std::vector<std::pair<uint32_t, float>> &weight, const EVICT_POLICY policy,
    uint64_t tick) {
  float temp;

  weight.reserve(blocks.size());

  switch (policy) {
    case POLICY_GREEDY:
    case POLICY_RANDOM:
    case POLICY_DCHOICE:
      for (auto &iter : blocks) {
        if (iter.second.getNextWritePageIndex() != param.pagesInBlock) {
          continue;
        }

        weight.push_back({iter.first, iter.second.getValidPageCountRaw()});
      }

      break;
    case POLICY_COST_BENEFIT:
      for (auto &iter : blocks) {
        if (iter.second.getNextWritePageIndex() != param.pagesInBlock) {
          continue;
        }

        temp = (float)(iter.second.getValidPageCountRaw()) / param.pagesInBlock;

        weight.push_back(
            {iter.first,
             temp / ((1 - temp) * (tick - iter.second.getLastAccessedTime()))});
      }

      break;
    default:
      panic("Invalid evict policy");
  }
}

void PageMapping::selectVictimBlock(std::vector<uint32_t> &list,
                                    uint64_t &tick) {
  static const GC_MODE mode = (GC_MODE)conf.readInt(CONFIG_FTL, FTL_GC_MODE);
  static const EVICT_POLICY policy =
      (EVICT_POLICY)conf.readInt(CONFIG_FTL, FTL_GC_EVICT_POLICY);
  static uint32_t dChoiceParam =
      conf.readUint(CONFIG_FTL, FTL_GC_D_CHOICE_PARAM);
  uint64_t nBlocks = conf.readUint(CONFIG_FTL, FTL_GC_RECLAIM_BLOCK);
  std::vector<std::pair<uint32_t, float>> weight;

  list.clear();

  // Calculate number of blocks to reclaim
  if (mode == GC_MODE_0) {
    // DO NOTHING
  }
  else if (mode == GC_MODE_1) {
    static const float t = conf.readFloat(CONFIG_FTL, FTL_GC_RECLAIM_THRESHOLD);

    nBlocks = param.totalPhysicalBlocks * t - nFreeBlocks;
  }
  else {
    panic("Invalid GC mode");
  }

  // reclaim one more if last free block fully used
  if (bReclaimMore) {
    nBlocks += param.pageCountToMaxPerf;

    bReclaimMore = false;
  }

  // Calculate weights of all blocks
  calculateVictimWeight(weight, policy, tick);

  if (policy == POLICY_RANDOM || policy == POLICY_DCHOICE) {
    uint64_t randomRange =
        policy == POLICY_RANDOM ? nBlocks : dChoiceParam * nBlocks;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, weight.size() - 1);
    std::vector<std::pair<uint32_t, float>> selected;

    while (selected.size() < randomRange) {
      uint64_t idx = dist(gen);

      if (weight.at(idx).first < std::numeric_limits<uint32_t>::max()) {
        selected.push_back(weight.at(idx));
        weight.at(idx).first = std::numeric_limits<uint32_t>::max();
      }
    }

    weight = std::move(selected);
  }

  // Sort weights
  std::sort(
      weight.begin(), weight.end(),
      [](std::pair<uint32_t, float> a, std::pair<uint32_t, float> b) -> bool {
        return a.second < b.second;
      });

  // Select victims from the blocks with the lowest weight
  nBlocks = MIN(nBlocks, weight.size());

  for (uint64_t i = 0; i < nBlocks; i++) {
    list.push_back(weight.at(i).first);
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::SELECT_VICTIM_BLOCK);
}

void PageMapping::doGarbageCollection(std::vector<uint32_t> &blocksToReclaim,
                                      uint64_t &tick) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<PAL::Request> readRequests;
  std::vector<PAL::Request> writeRequests;
  std::vector<PAL::Request> eraseRequests;
  std::vector<uint64_t> lpns;
  Bitset bit(param.ioUnitInPage);
  uint64_t beginAt;
  uint64_t readFinishedAt = tick;
  uint64_t writeFinishedAt = tick;
  uint64_t eraseFinishedAt = tick;

  if (blocksToReclaim.size() == 0) {
    return;
  }

  // For all blocks to reclaim, collecting request structure only
  for (auto &iter : blocksToReclaim) {
    auto block = blocks.find(iter);

    if (block == blocks.end()) {
      panic("Invalid block");
    }

    // Copy valid pages to free block
    for (uint32_t pageIndex = 0; pageIndex < param.pagesInBlock; pageIndex++) {
      // Valid?
      if (block->second.getPageInfo(pageIndex, lpns, bit)) {
        if (!bRandomTweak) {
          bit.set();
        }

        // Retrive free block
        auto freeBlock = blocks.find(getLastFreeBlock2(bit, 0));

        // Issue Read
        req.blockIndex = block->first;
        req.pageIndex = pageIndex;
        req.ioFlag = bit;

        readRequests.push_back(req);

        // Update mapping table
        uint32_t newBlockIdx = freeBlock->first;

        for (uint32_t idx = 0; idx < bitsetSize; idx++) {
          if (bit.test(idx)) {
            // Invalidate
            block->second.invalidate(pageIndex, idx);

            auto mappingList = table.find(lpns.at(idx));

            if (mappingList == table.end()) {
              panic("Invalid mapping table entry");
            }

            pDRAM->read(&(*mappingList), 8 * param.ioUnitInPage, tick);

            auto &mapping = mappingList->second.at(idx);

            uint32_t newPageIdx = freeBlock->second.getNextWritePageIndex(idx);

            mapping.first = newBlockIdx;
            mapping.second = newPageIdx;

            freeBlock->second.write(newPageIdx, lpns.at(idx), idx, beginAt);

            // Issue Write
            req.blockIndex = newBlockIdx;
            req.pageIndex = newPageIdx;

            if (bRandomTweak) {
              req.ioFlag.reset();
              req.ioFlag.set(idx);
            }
            else {
              req.ioFlag.set();
            }

            writeRequests.push_back(req);

            stat.validPageCopies++;
          }
        }

        stat.validSuperPageCopies++;
      }
    }

    // Erase block
    req.blockIndex = block->first;
    req.pageIndex = 0;
    req.ioFlag.set();

    eraseRequests.push_back(req);
  }

  // Do actual I/O here
  // This handles PAL2 limitation (SIGSEGV, infinite loop, or so-on)
  for (auto &iter : readRequests) {
    beginAt = tick;

    pPAL->read(iter, beginAt);

    readFinishedAt = MAX(readFinishedAt, beginAt);
  }

  for (auto &iter : writeRequests) {
    beginAt = readFinishedAt;

    pPAL->write(iter, beginAt);

    writeFinishedAt = MAX(writeFinishedAt, beginAt);
  }

  for (auto &iter : eraseRequests) {
    beginAt = readFinishedAt;

    //eraseInternal(iter, beginAt);
    eraseInternal2(iter, beginAt);
    
    eraseFinishedAt = MAX(eraseFinishedAt, beginAt);
  }

  tick = MAX(writeFinishedAt, eraseFinishedAt);
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::DO_GARBAGE_COLLECTION);
}

void PageMapping::readInternal(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
    }

    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          palRequest.blockIndex = mapping.first;
          palRequest.pageIndex = mapping.second;

          if (bRandomTweak) {
            palRequest.ioFlag.reset();
            palRequest.ioFlag.set(idx);
          }
          else {
            palRequest.ioFlag.set();
          }

          auto block = blocks.find(palRequest.blockIndex);

          if (block == blocks.end()) {
            panic("Block is not in use");
          }

          beginAt = tick;

          block->second.read(palRequest.pageIndex, idx, beginAt);
          pPAL->read(palRequest, beginAt);

          finishedAt = MAX(finishedAt, beginAt);
        }
      }
    }

    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
  }
}

void PageMapping::readInternal2(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  static const WL_POLICY wlMode =
      (WL_POLICY)conf.readInt(CONFIG_FTL, FTL_WL_MODE);
  
  if (wlMode == WL_DYNAMIC)
  {
    updateLRU(req.lpn);
  }

  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
    }

    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          palRequest.blockIndex = mapping.first;
          palRequest.pageIndex = mapping.second;

          if (bRandomTweak) {
            palRequest.ioFlag.reset();
            palRequest.ioFlag.set(idx);
          }
          else {
            palRequest.ioFlag.set();
          }

          auto block = blocks.find(palRequest.blockIndex);

          if (block == blocks.end()) {
            panic("Block is not in use");
          }

          beginAt = tick;

          block->second.read(palRequest.pageIndex, idx, beginAt);
          pPAL->read(palRequest, beginAt);

          finishedAt = MAX(finishedAt, beginAt);
        }
      }
    }

    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
  }
}

void PageMapping::writeInternal(Request &req, uint64_t &tick, bool sendToPAL) {
  PAL::Request palRequest(req);
  std::unordered_map<uint32_t, Block>::iterator block;
  auto mappingList = table.find(req.lpn);
  uint64_t beginAt;
  uint64_t finishedAt = tick;
  bool readBeforeWrite = false;

  if (mappingList != table.end()) {
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          block = blocks.find(mapping.first);

          // Invalidate current page
          block->second.invalidate(mapping.second, idx);
        }
      }
    }
  }
  else {
    // Create empty mapping
    auto ret = table.emplace(
        req.lpn,
        std::vector<std::pair<uint32_t, uint32_t>>(
            bitsetSize, {param.totalPhysicalBlocks, param.pagesInBlock}));

    if (!ret.second) {
      panic("Failed to insert new mapping");
    }

    mappingList = ret.first;
  }

  // Write data to free block
  block = blocks.find(getLastFreeBlock(req.ioFlag));

  if (block == blocks.end()) {
    panic("No such block");
  }

  if (sendToPAL) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
      pDRAM->write(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
      pDRAM->write(&(*mappingList), 8, tick);
    }
  }

  if (!bRandomTweak && !req.ioFlag.all()) {
    // We have to read old data
    readBeforeWrite = true;
  }

  for (uint32_t idx = 0; idx < bitsetSize; idx++) {
    if (req.ioFlag.test(idx) || !bRandomTweak) {
      uint32_t pageIndex = block->second.getNextWritePageIndex(idx);
      auto &mapping = mappingList->second.at(idx);

      beginAt = tick;

      block->second.write(pageIndex, req.lpn, idx, beginAt);

      // Read old data if needed (Only executed when bRandomTweak = false)
      // Maybe some other init procedures want to perform 'partial-write'
      // So check sendToPAL variable
      if (readBeforeWrite && sendToPAL) {
        palRequest.blockIndex = mapping.first;
        palRequest.pageIndex = mapping.second;

        // We don't need to read old data
        palRequest.ioFlag = req.ioFlag;
        palRequest.ioFlag.flip();

        pPAL->read(palRequest, beginAt);
      }

      // update mapping to table
      mapping.first = block->first;
      mapping.second = pageIndex;

      if (sendToPAL) {
        palRequest.blockIndex = block->first;
        palRequest.pageIndex = pageIndex;

        if (bRandomTweak) {
          palRequest.ioFlag.reset();
          palRequest.ioFlag.set(idx);
        }
        else {
          palRequest.ioFlag.set();
        }

        pPAL->write(palRequest, beginAt);
      }

      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  // Exclude CPU operation when initializing
  if (sendToPAL) {
    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE_INTERNAL);
  }

  // GC if needed
  // I assumed that init procedure never invokes GC
  static float gcThreshold = conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO);

  if (freeBlockRatio() < gcThreshold) {
    if (!sendToPAL) {
      panic("ftl: GC triggered while in initialization");
    }

    std::vector<uint32_t> list;
    uint64_t beginAt = tick;

    selectVictimBlock(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | On-demand | %u blocks will be reclaimed", list.size());

    doGarbageCollection(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | Done | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")", tick,
               beginAt, beginAt - tick);

    stat.gcCount++;
    stat.reclaimedBlocks += list.size();
  }
}

void PageMapping::writeInternal2(Request &req, uint64_t &tick, bool sendToPAL) {
  PAL::Request palRequest(req);
  std::unordered_map<uint32_t, Block>::iterator block;
  auto mappingList = table.find(req.lpn);
  uint64_t beginAt;
  uint64_t finishedAt = tick;
  bool readBeforeWrite = false;

  // BER modeling data  
  static uint64_t eraseThreshold =
      conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  static float initialBER =
      conf.readFloat(CONFIG_FTL, FTL_INITIAL_BER);
  static float finalBER =
      conf.readFloat(CONFIG_FTL, FTL_FINAL_BER);
  static float sigma =
      conf.readFloat(CONFIG_FTL, FTL_BER_SIGMA);

  // BER modeling random generator
  std::random_device generator;
  uint32_t errorCount;
  float averageError;
  float temp;
  uint32_t isHot;

  static const WL_POLICY wlMode =
        (WL_POLICY)conf.readInt(CONFIG_FTL, FTL_WL_MODE);
  
  if (wlMode == WL_GREEDY)
  {
    isHot = 1;
  }
  else
  {
    isHot = updateLRU(req.lpn);
  }
  //updateLRU(req.lpn);

  



  if (mappingList != table.end()) {
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        auto &mapping = mappingList->second.at(idx);

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          block = blocks.find(mapping.first);

          // Invalidate current page
          block->second.invalidate(mapping.second, idx);
        }
      }
    }
  }
  else {
    // Create empty mapping
    auto ret = table.emplace(
        req.lpn,
        std::vector<std::pair<uint32_t, uint32_t>>(
            bitsetSize, {param.totalPhysicalBlocks, param.pagesInBlock}));

    if (!ret.second) {
      panic("Failed to insert new mapping");
    }

    mappingList = ret.first;
  }

  // Write data to free block
  block = blocks.find(getLastFreeBlock2(req.ioFlag, isHot));

  if (block == blocks.end()) {
    panic("No such block");
  }

  if (sendToPAL) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
      pDRAM->write(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
      pDRAM->write(&(*mappingList), 8, tick);
    }
  }

  if (!bRandomTweak && !req.ioFlag.all()) {
    // We have to read old data
    readBeforeWrite = true;
  }

  for (uint32_t idx = 0; idx < bitsetSize; idx++) {
    if (req.ioFlag.test(idx) || !bRandomTweak) {
      uint32_t pageIndex = block->second.getNextWritePageIndex(idx);
      auto &mapping = mappingList->second.at(idx);

      beginAt = tick;

      block->second.write(pageIndex, req.lpn, idx, beginAt);

      // Read old data if needed (Only executed when bRandomTweak = false)
      // Maybe some other init procedures want to perform 'partial-write'
      // So check sendToPAL variable
      if (readBeforeWrite && sendToPAL) {
        palRequest.blockIndex = mapping.first;
        palRequest.pageIndex = mapping.second;

        // We don't need to read old data
        palRequest.ioFlag = req.ioFlag;
        palRequest.ioFlag.flip();

        pPAL->read(palRequest, beginAt);
      }

      // update mapping to table
      mapping.first = block->first;
      mapping.second = pageIndex;

      if (sendToPAL) {
        palRequest.blockIndex = block->first;
        palRequest.pageIndex = pageIndex;

        if (bRandomTweak) {
          palRequest.ioFlag.reset();
          palRequest.ioFlag.set(idx);
        }
        else {
          palRequest.ioFlag.set();
        }

        pPAL->write(palRequest, beginAt);
        pPAL->read(palRequest, beginAt);
      }

      averageError = (finalBER - initialBER)/eraseThreshold
                        * (block->second.getEraseCount()) + initialBER;
      averageError = averageError * param.pageSize * 8;
      std::normal_distribution<double> normal(averageError,sigma);

      // Remove negative value
      temp = normal(generator);
      if (temp < 0){
        temp = 0;
      }

      // Keep maximum error count ever seen
      errorCount = MAX(static_cast<uint32_t>(temp + 0.5), block->second.getBitErrorCount());
      //if(errorCount>=128){
      //  errorCount = 127;
      //}
      block->second.updateError(errorCount);


      finishedAt = MAX(finishedAt, beginAt);
    }
  }

  // Exclude CPU operation when initializing
  if (sendToPAL) {
    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE_INTERNAL);
    //tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
  }

  // GC if needed
  // I assumed that init procedure never invokes GC
  static float gcThreshold = conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO);

  if (freeBlockRatio() < gcThreshold) {
    if (!sendToPAL) {
      panic("ftl: GC triggered while in initialization");
    }

    std::vector<uint32_t> list;
    uint64_t beginAt = tick;

    selectVictimBlock(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | On-demand | %u blocks will be reclaimed", list.size());

    doGarbageCollection(list, beginAt);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "GC   | Done | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")", tick,
               beginAt, beginAt - tick);

    stat.gcCount++;
    stat.reclaimedBlocks += list.size();
  }
}

void PageMapping::trimInternal(Request &req, uint64_t &tick) {
  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
    }

    // Do trim
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      auto &mapping = mappingList->second.at(idx);
      auto block = blocks.find(mapping.first);

      if (block == blocks.end()) {
        panic("Block is not in use");
      }

      block->second.invalidate(mapping.second, idx);
    }

    // Remove mapping
    table.erase(mappingList);

    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM_INTERNAL);
  }
}

void PageMapping::eraseInternal(PAL::Request &req, uint64_t &tick) {
  static uint64_t threshold =
      conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(req.blockIndex);

  // Sanity checks
  if (block == blocks.end()) {
    panic("No such block");
  }

  if (block->second.getValidPageCount() != 0) {
    panic("There are valid pages in victim block");
  }

  // Erase block
  block->second.erase();

  pPAL->erase(req, tick);

  // Check erase count
  uint32_t erasedCount = block->second.getEraseCount();

  if (erasedCount < threshold) {
    // Reverse search
    auto iter = freeBlocks.end();

    while (true) {
      iter--;

      if (iter->getEraseCount() <= erasedCount) {
        // emplace: insert before pos
        iter++;

        break;
      }

      if (iter == freeBlocks.begin()) {
        break;
      }
    }

    // Insert block to free block list
    freeBlocks.emplace(iter, std::move(block->second));
    nFreeBlocks++;
  }

  // Remove block from block list
  blocks.erase(block);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::ERASE_INTERNAL);
}

void PageMapping::eraseInternal2(PAL::Request &req, uint64_t &tick) {
  //static uint64_t threshold =
  //    conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(req.blockIndex);

  // Sanity checks
  if (block == blocks.end()) {
    panic("No such block");
  }

  if (block->second.getValidPageCount() != 0) {
    panic("There are valid pages in victim block");
  }


  // Check error count
  uint32_t blockIdx = block->second.getBlockIndex();
  uint32_t errorCount = block->second.getBitErrorCount();
  uint32_t errorTableIndex = block->second.getErrorTableIndex();
  


  // Erase block
  block->second.erase();

  pPAL->erase(req, tick);


  //uint32_t erasedCount = block->second.getEraseCount();
  uint32_t eccCapability = conf.readInt(CONFIG_FTL, FTL_ECC_CAPABILITY);

  // If error count is over ECC capability, the block will not be used any more.
  if (errorCount < eccCapability) {
    // Push to back of free list
    auto tableIter = errorCountTable.begin() + errorCount;
    tableIter->first.emplace_back(std::move(block->second));
    badBlockCount ++;
  }
  else{
    // for stat
    errorCount = 127;
  }

  // Remove block from available block list
  // First find block from valid list
  auto iter = errorCountTable[errorTableIndex].second.begin();

  while(iter != errorCountTable[errorTableIndex].second.end()){
    if(*iter == blockIdx){
      break;
    }
    iter++;
  }

  // Sanity check
  if (iter == errorCountTable[errorTableIndex].second.end())
  {
    panic("Block is lost in availabe list");
  }
  // Erase from valid list and metadata list
  iter = errorCountTable[errorTableIndex].second.erase(iter);
  blocks.erase(block);

  nFreeBlocks++;

  // Store block counts for each number of errors.
  errorCountStat[errorTableIndex] --;
  errorCountStat[errorCount] ++;

  // The block becomes vulnerable block when error count is over weak boundary
  if (errorCount > wearLevelingStat.weakBoundary)
    {
    vulBlockCount ++;
    // If too many vulnerable blocks, increament weark boundary
    while (vulBlockCount > (param.totalPhysicalBlocks - badBlockCount) / 10)
    {
      wearLevelingStat.weakBoundary ++;
      vulBlockCount = vulBlockCount - errorCountStat[wearLevelingStat.weakBoundary];
    }
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::ERASE_INTERNAL);
}

uint32_t PageMapping::updateLRU(uint32_t lpn){
  // Find lpn in lru window
  auto lruIter = lruWindow.begin();
  uint32_t lbn = lpn / param.totalLogicalBlocks;
  uint32_t isHot = 0;  // 1: Hot, 0: Cold

  while (lruIter != lruWindow.end())
  {
    if (lruIter->first == lbn)
    {
      break;
    }
    lruIter++;
  }

  if(lruIter != lruWindow.end()){
    // Found in window, erase and push to MRU position (back of window)
    uint32_t newCount = lruIter->second + 1;
    std::pair<uint32_t, uint32_t> newEntry 
        = std::make_pair(lruIter->first, newCount);

    lruIter = lruWindow.erase(lruIter);
    // Update average
    uint32_t countSum = 0;
    lruWindow.push_back(newEntry);
    for(auto &entry : lruWindow){
        countSum = countSum + entry.second;
      }
    lruAverage = countSum / lruWindow.size();
    
    if (newCount > lruAverage){
      isHot = 1;
    }
    
  }
  else{
    // Not found in window, push new one to MRU position (back of window)
    std::pair<uint32_t, uint32_t> newEntry 
        = std::make_pair(lbn, 1);
    lruWindow.push_back(newEntry);

    // If window is full, evict LRU position entry, halve the counters
    if(lruWindowSize < lruWindow.size()){
      lruWindow.pop_front();
      
      uint32_t countSum = 0;
      for(auto &entry : lruWindow){
        entry.second = entry.second / 2;
        countSum = countSum + entry.second;
      }
      lruAverage = countSum / lruWindow.size();
      // New block is always cold
    }
  }

  if (isHot == 1)
  {
    hotCount ++;
  }
  else{
    coldCount ++;
  }
  

  return isHot;
}

float PageMapping::calculateWearLeveling() {
  uint64_t totalEraseCnt = 0;
  uint64_t sumOfSquaredEraseCnt = 0;
  uint64_t numOfBlocks = param.totalLogicalBlocks;
  uint64_t eraseCnt;

  for (auto &iter : blocks) {
    eraseCnt = iter.second.getEraseCount();
    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  // freeBlocks is sorted
  // Calculate from backward, stop when eraseCnt is zero
  for (auto riter = freeBlocks.rbegin(); riter != freeBlocks.rend(); riter++) {
    eraseCnt = riter->getEraseCount();

    if (eraseCnt == 0) {
      break;
    }

    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  if (sumOfSquaredEraseCnt == 0) {
    return -1;  // no meaning of wear-leveling
  }

  return (float)totalEraseCnt * totalEraseCnt /
         (numOfBlocks * sumOfSquaredEraseCnt);
}

void PageMapping::calculateTotalPages(uint64_t &valid, uint64_t &invalid) {
  valid = 0;
  invalid = 0;

  for (auto &iter : blocks) {
    valid += iter.second.getValidPageCount();
    invalid += iter.second.getDirtyPageCount();
  }
}

void PageMapping::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "page_mapping.gc.count";
  temp.desc = "Total GC count";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.reclaimed_blocks";
  temp.desc = "Total reclaimed blocks in GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.superpage_copies";
  temp.desc = "Total copied valid superpages during GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.page_copies";
  temp.desc = "Total copied valid pages during GC";
  list.push_back(temp);

  // For the exact definition, see following paper:
  // Li, Yongkun, Patrick PC Lee, and John Lui.
  // "Stochastic modeling of large-scale solid-state storage systems: analysis,
  // design tradeoffs and optimization." ACM SIGMETRICS (2013)
  temp.name = prefix + "page_mapping.wear_leveling";
  temp.desc = "Wear-leveling factor";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.LRU_size";
  temp.desc = "number of blocks in LRU window";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.LRU_average";
  temp.desc = "LRU window counter average";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.LRU_hot_count";
  temp.desc = "LRU hot data count";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.LRU_cold_count";
  temp.desc = "LRU cold data count";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.strong_boundary";
  temp.desc = "Strong boundary";
  list.push_back(temp);

  for(uint32_t i = 0; i < 128; i++){
        temp.name = prefix + "page_mapping.error_stat";
        temp.desc = "Error count :" + std::to_string(i);
        list.push_back(temp);
  }
}

void PageMapping::getStatValues(std::vector<double> &values) {
  values.push_back(stat.gcCount);
  values.push_back(stat.reclaimedBlocks);
  values.push_back(stat.validSuperPageCopies);
  values.push_back(stat.validPageCopies);
  values.push_back(calculateWearLeveling());
  values.push_back(lruWindow.size());
  values.push_back(lruAverage);
  values.push_back(hotCount);
  values.push_back(coldCount);
  values.push_back(wearLevelingStat.strongBoundary);

  for(uint32_t i = 0; i < 128; i++){
    values.push_back(errorCountStat[i]);
  }
}

void PageMapping::resetStatValues() {
  memset(&stat, 0, sizeof(stat));
}

}  // namespace FTL

}  // namespace SimpleSSD
