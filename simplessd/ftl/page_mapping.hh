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

#ifndef __FTL_PAGE_MAPPING__
#define __FTL_PAGE_MAPPING__

#include <cinttypes>
#include <unordered_map>
#include <vector>

#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"

namespace SimpleSSD {

namespace FTL {

class PageMapping : public AbstractFTL {
 private:
  PAL::PAL *pPAL;

  ConfigReader &conf;

  std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>>
      table;
  std::unordered_map<uint32_t, Block> blocks;
  std::list<Block> freeBlocks;
  uint32_t nFreeBlocks;  // For some libraries which std::list::size() is O(n)
  std::vector<uint32_t> lastFreeBlock;
  Bitset lastFreeBlockIOMap;
  uint32_t lastFreeBlockIndex;


  bool bReclaimMore;
  bool bRandomTweak;
  uint32_t bitsetSize;

  struct {
    uint64_t gcCount;
    uint64_t reclaimedBlocks;
    uint64_t validSuperPageCopies;
    uint64_t validPageCopies;
  } stat;

  // Stat for wear leveling
  struct {
    uint16_t maxErrorCount;
    uint16_t minErrorCount;
    uint16_t strongBoundary;
    uint16_t weakBoundary;
  } wearLevelingStat;

  // Table for handle blocks with # of errors
  std::vector<std::pair<std::list<Block>, std::list<uint32_t>>>
      errorCountTable;

  std::vector<uint32_t> errorCountStat;
  
  std::vector<vector<uint32_t>> lastFreeBlock2;
  std::vector<Bitset> lastFreeBlockIOMap2;
  std::vector<uint32_t> lastFreeBlockIndex2;

  // LRU window for hot/cold identification
  // list of (LBA, access count)
  std::list<std::pair<uint32_t, uint32_t>> lruWindow;
  uint32_t lruAverage;
  uint32_t lruWindowSize;
  uint32_t vulBlockCount;
  uint32_t badBlockCount;
  uint32_t testCount = 0;

  // BER data
  double initialBER;
  double finalBER;

  float freeBlockRatio();
  uint32_t convertBlockIdx(uint32_t);
  uint32_t getFreeBlock(uint32_t);
  uint32_t getLastFreeBlock(Bitset &);
  void calculateVictimWeight(std::vector<std::pair<uint32_t, float>> &,
                             const EVICT_POLICY, uint64_t);
  void selectVictimBlock(std::vector<uint32_t> &, uint64_t &);
  void doGarbageCollection(std::vector<uint32_t> &, uint64_t &);

  float calculateWearLeveling();
  void calculateTotalPages(uint64_t &, uint64_t &);

  void readInternal(Request &, uint64_t &);
  void writeInternal(Request &, uint64_t &, bool = true);
  void trimInternal(Request &, uint64_t &);
  void eraseInternal(PAL::Request &, uint64_t &);

  // Page mapping scheme for BER wearleveling
  uint32_t getFreeBlock2(uint16_t, uint32_t);
  uint32_t getLastFreeBlock2(Bitset&, uint32_t);
  void eraseInternal2(PAL::Request &, uint64_t &);

  void readInternal2(Request &, uint64_t &);
  void writeInternal2(Request &, uint64_t &, bool = true);
  uint32_t updateLRU(uint32_t);

 public:
  PageMapping(ConfigReader &, Parameter &, PAL::PAL *, DRAM::AbstractDRAM *);
  ~PageMapping();

  bool initialize() override;

  void read(Request &, uint64_t &) override;
  void write(Request &, uint64_t &) override;
  void trim(Request &, uint64_t &) override;

  void format(LPNRange &, uint64_t &) override;

  Status *getStatus(uint64_t, uint64_t) override;

  void getStatList(std::vector<Stats> &, std::string) override;
  void getStatValues(std::vector<double> &) override;
  void resetStatValues() override;
};

}  // namespace FTL

}  // namespace SimpleSSD

#endif
