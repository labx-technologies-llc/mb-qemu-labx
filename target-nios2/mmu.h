/*
 *  Altera Nios II MMU emulation for qemu.
 *
 *  Copyright (c) 2012 Chris Wulff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

struct nios2_tlb_entry {
  target_ulong tag;
  target_ulong data;
};

struct nios2_mmu {
  int pid_bits;
  int tlb_num_ways;
  int tlb_num_entries;
  int tlb_entry_mask;
  struct nios2_tlb_entry *tlb;
};

struct nios2_mmu_lookup {
  target_ulong vaddr;
  target_ulong paddr;
  int prot;
};

void mmu_flip_um(CPUState *env, unsigned int um);
unsigned int mmu_translate(CPUState *env,
                           struct nios2_mmu_lookup *lu,
                           target_ulong vaddr, int rw, int mmu_idx);
uint32_t mmu_read(CPUState *env, uint32_t rn);
void mmu_write(CPUState *env, uint32_t rn, uint32_t v);
void mmu_init(struct nios2_mmu *mmu);
