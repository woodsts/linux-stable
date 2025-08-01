.. SPDX-License-Identifier: GPL-2.0

===============================
Architecture Page Table Helpers
===============================

Generic MM expects architectures (with MMU) to provide helpers to create, access
and modify page table entries at various level for different memory functions.
These page table helpers need to conform to a common semantics across platforms.
Following tables describe the expected semantics which can also be tested during
boot via CONFIG_DEBUG_VM_PGTABLE option. All future changes in here or the debug
test need to be in sync.


PTE Page Table Helpers
======================

+---------------------------+--------------------------------------------------+
| pte_same                  | Tests whether both PTE entries are the same      |
+---------------------------+--------------------------------------------------+
| pte_present               | Tests a valid mapped PTE                         |
+---------------------------+--------------------------------------------------+
| pte_young                 | Tests a young PTE                                |
+---------------------------+--------------------------------------------------+
| pte_dirty                 | Tests a dirty PTE                                |
+---------------------------+--------------------------------------------------+
| pte_write                 | Tests a writable PTE                             |
+---------------------------+--------------------------------------------------+
| pte_special               | Tests a special PTE                              |
+---------------------------+--------------------------------------------------+
| pte_protnone              | Tests a PROT_NONE PTE                            |
+---------------------------+--------------------------------------------------+
| pte_soft_dirty            | Tests a soft dirty PTE                           |
+---------------------------+--------------------------------------------------+
| pte_swp_soft_dirty        | Tests a soft dirty swapped PTE                   |
+---------------------------+--------------------------------------------------+
| pte_mkyoung               | Creates a young PTE                              |
+---------------------------+--------------------------------------------------+
| pte_mkold                 | Creates an old PTE                               |
+---------------------------+--------------------------------------------------+
| pte_mkdirty               | Creates a dirty PTE                              |
+---------------------------+--------------------------------------------------+
| pte_mkclean               | Creates a clean PTE                              |
+---------------------------+--------------------------------------------------+
| pte_mkwrite               | Creates a writable PTE of the type specified by  |
|                           | the VMA.                                         |
+---------------------------+--------------------------------------------------+
| pte_mkwrite_novma         | Creates a writable PTE, of the conventional type |
|                           | of writable.                                     |
+---------------------------+--------------------------------------------------+
| pte_wrprotect             | Creates a write protected PTE                    |
+---------------------------+--------------------------------------------------+
| pte_mkspecial             | Creates a special PTE                            |
+---------------------------+--------------------------------------------------+
| pte_mkdevmap              | Creates a ZONE_DEVICE mapped PTE                 |
+---------------------------+--------------------------------------------------+
| pte_mksoft_dirty          | Creates a soft dirty PTE                         |
+---------------------------+--------------------------------------------------+
| pte_clear_soft_dirty      | Clears a soft dirty PTE                          |
+---------------------------+--------------------------------------------------+
| pte_swp_mksoft_dirty      | Creates a soft dirty swapped PTE                 |
+---------------------------+--------------------------------------------------+
| pte_swp_clear_soft_dirty  | Clears a soft dirty swapped PTE                  |
+---------------------------+--------------------------------------------------+
| pte_mknotpresent          | Invalidates a mapped PTE                         |
+---------------------------+--------------------------------------------------+
| ptep_clear                | Clears a PTE                                     |
+---------------------------+--------------------------------------------------+
| ptep_get_and_clear        | Clears and returns PTE                           |
+---------------------------+--------------------------------------------------+
| ptep_get_and_clear_full   | Clears and returns PTE (batched PTE unmap)       |
+---------------------------+--------------------------------------------------+
| ptep_test_and_clear_young | Clears young from a PTE                          |
+---------------------------+--------------------------------------------------+
| ptep_set_wrprotect        | Converts into a write protected PTE              |
+---------------------------+--------------------------------------------------+
| ptep_set_access_flags     | Converts into a more permissive PTE              |
+---------------------------+--------------------------------------------------+


PMD Page Table Helpers
======================

+---------------------------+--------------------------------------------------+
| pmd_same                  | Tests whether both PMD entries are the same      |
+---------------------------+--------------------------------------------------+
| pmd_bad                   | Tests a non-table mapped PMD                     |
+---------------------------+--------------------------------------------------+
| pmd_leaf                  | Tests a leaf mapped PMD                          |
+---------------------------+--------------------------------------------------+
| pmd_trans_huge            | Tests a Transparent Huge Page (THP) at PMD       |
+---------------------------+--------------------------------------------------+
| pmd_present               | Tests whether pmd_page() points to valid memory  |
+---------------------------+--------------------------------------------------+
| pmd_young                 | Tests a young PMD                                |
+---------------------------+--------------------------------------------------+
| pmd_dirty                 | Tests a dirty PMD                                |
+---------------------------+--------------------------------------------------+
| pmd_write                 | Tests a writable PMD                             |
+---------------------------+--------------------------------------------------+
| pmd_special               | Tests a special PMD                              |
+---------------------------+--------------------------------------------------+
| pmd_protnone              | Tests a PROT_NONE PMD                            |
+---------------------------+--------------------------------------------------+
| pmd_soft_dirty            | Tests a soft dirty PMD                           |
+---------------------------+--------------------------------------------------+
| pmd_swp_soft_dirty        | Tests a soft dirty swapped PMD                   |
+---------------------------+--------------------------------------------------+
| pmd_mkyoung               | Creates a young PMD                              |
+---------------------------+--------------------------------------------------+
| pmd_mkold                 | Creates an old PMD                               |
+---------------------------+--------------------------------------------------+
| pmd_mkdirty               | Creates a dirty PMD                              |
+---------------------------+--------------------------------------------------+
| pmd_mkclean               | Creates a clean PMD                              |
+---------------------------+--------------------------------------------------+
| pmd_mkwrite               | Creates a writable PMD of the type specified by  |
|                           | the VMA.                                         |
+---------------------------+--------------------------------------------------+
| pmd_mkwrite_novma         | Creates a writable PMD, of the conventional type |
|                           | of writable.                                     |
+---------------------------+--------------------------------------------------+
| pmd_wrprotect             | Creates a write protected PMD                    |
+---------------------------+--------------------------------------------------+
| pmd_mkspecial             | Creates a special PMD                            |
+---------------------------+--------------------------------------------------+
| pmd_mkdevmap              | Creates a ZONE_DEVICE mapped PMD                 |
+---------------------------+--------------------------------------------------+
| pmd_mksoft_dirty          | Creates a soft dirty PMD                         |
+---------------------------+--------------------------------------------------+
| pmd_clear_soft_dirty      | Clears a soft dirty PMD                          |
+---------------------------+--------------------------------------------------+
| pmd_swp_mksoft_dirty      | Creates a soft dirty swapped PMD                 |
+---------------------------+--------------------------------------------------+
| pmd_swp_clear_soft_dirty  | Clears a soft dirty swapped PMD                  |
+---------------------------+--------------------------------------------------+
| pmd_mkinvalid             | Invalidates a present PMD; do not call for       |
|                           | non-present PMD [1]                              |
+---------------------------+--------------------------------------------------+
| pmd_set_huge              | Creates a PMD huge mapping                       |
+---------------------------+--------------------------------------------------+
| pmd_clear_huge            | Clears a PMD huge mapping                        |
+---------------------------+--------------------------------------------------+
| pmdp_get_and_clear        | Clears a PMD                                     |
+---------------------------+--------------------------------------------------+
| pmdp_get_and_clear_full   | Clears a PMD                                     |
+---------------------------+--------------------------------------------------+
| pmdp_test_and_clear_young | Clears young from a PMD                          |
+---------------------------+--------------------------------------------------+
| pmdp_set_wrprotect        | Converts into a write protected PMD              |
+---------------------------+--------------------------------------------------+
| pmdp_set_access_flags     | Converts into a more permissive PMD              |
+---------------------------+--------------------------------------------------+


PUD Page Table Helpers
======================

+---------------------------+--------------------------------------------------+
| pud_same                  | Tests whether both PUD entries are the same      |
+---------------------------+--------------------------------------------------+
| pud_bad                   | Tests a non-table mapped PUD                     |
+---------------------------+--------------------------------------------------+
| pud_leaf                  | Tests a leaf mapped PUD                          |
+---------------------------+--------------------------------------------------+
| pud_trans_huge            | Tests a Transparent Huge Page (THP) at PUD       |
+---------------------------+--------------------------------------------------+
| pud_present               | Tests a valid mapped PUD                         |
+---------------------------+--------------------------------------------------+
| pud_young                 | Tests a young PUD                                |
+---------------------------+--------------------------------------------------+
| pud_dirty                 | Tests a dirty PUD                                |
+---------------------------+--------------------------------------------------+
| pud_write                 | Tests a writable PUD                             |
+---------------------------+--------------------------------------------------+
| pud_mkyoung               | Creates a young PUD                              |
+---------------------------+--------------------------------------------------+
| pud_mkold                 | Creates an old PUD                               |
+---------------------------+--------------------------------------------------+
| pud_mkdirty               | Creates a dirty PUD                              |
+---------------------------+--------------------------------------------------+
| pud_mkclean               | Creates a clean PUD                              |
+---------------------------+--------------------------------------------------+
| pud_mkwrite               | Creates a writable PUD                           |
+---------------------------+--------------------------------------------------+
| pud_wrprotect             | Creates a write protected PUD                    |
+---------------------------+--------------------------------------------------+
| pud_mkdevmap              | Creates a ZONE_DEVICE mapped PUD                 |
+---------------------------+--------------------------------------------------+
| pud_mkinvalid             | Invalidates a present PUD; do not call for       |
|                           | non-present PUD [1]                              |
+---------------------------+--------------------------------------------------+
| pud_set_huge              | Creates a PUD huge mapping                       |
+---------------------------+--------------------------------------------------+
| pud_clear_huge            | Clears a PUD huge mapping                        |
+---------------------------+--------------------------------------------------+
| pudp_get_and_clear        | Clears a PUD                                     |
+---------------------------+--------------------------------------------------+
| pudp_get_and_clear_full   | Clears a PUD                                     |
+---------------------------+--------------------------------------------------+
| pudp_test_and_clear_young | Clears young from a PUD                          |
+---------------------------+--------------------------------------------------+
| pudp_set_wrprotect        | Converts into a write protected PUD              |
+---------------------------+--------------------------------------------------+
| pudp_set_access_flags     | Converts into a more permissive PUD              |
+---------------------------+--------------------------------------------------+


HugeTLB Page Table Helpers
==========================

+---------------------------+--------------------------------------------------+
| pte_huge                  | Tests a HugeTLB                                  |
+---------------------------+--------------------------------------------------+
| arch_make_huge_pte        | Creates a HugeTLB                                |
+---------------------------+--------------------------------------------------+
| huge_pte_dirty            | Tests a dirty HugeTLB                            |
+---------------------------+--------------------------------------------------+
| huge_pte_write            | Tests a writable HugeTLB                         |
+---------------------------+--------------------------------------------------+
| huge_pte_mkdirty          | Creates a dirty HugeTLB                          |
+---------------------------+--------------------------------------------------+
| huge_pte_mkwrite          | Creates a writable HugeTLB                       |
+---------------------------+--------------------------------------------------+
| huge_pte_wrprotect        | Creates a write protected HugeTLB                |
+---------------------------+--------------------------------------------------+
| huge_ptep_get_and_clear   | Clears a HugeTLB                                 |
+---------------------------+--------------------------------------------------+
| huge_ptep_set_wrprotect   | Converts into a write protected HugeTLB          |
+---------------------------+--------------------------------------------------+
| huge_ptep_set_access_flags  | Converts into a more permissive HugeTLB        |
+---------------------------+--------------------------------------------------+


SWAP Page Table Helpers
========================

+---------------------------+--------------------------------------------------+
| __pte_to_swp_entry        | Creates a swp_entry_t (arch) from a swap PTE     |
+---------------------------+--------------------------------------------------+
| __swp_entry_to_pte        | Creates a swap PTE from a swp_entry_t (arch)     |
+---------------------------+--------------------------------------------------+
| __pmd_to_swp_entry        | Creates a swp_entry_t (arch) from a swap PMD     |
+---------------------------+--------------------------------------------------+
| __swp_entry_to_pmd        | Creates a swap PMD from a swp_entry_t (arch)     |
+---------------------------+--------------------------------------------------+
| is_migration_entry        | Tests a migration (read or write) swapped entry  |
+-------------------------------+----------------------------------------------+
| is_writable_migration_entry   | Tests a write migration swapped entry        |
+-------------------------------+----------------------------------------------+
| make_readable_migration_entry | Creates a read migration swapped entry       |
+-------------------------------+----------------------------------------------+
| make_writable_migration_entry | Creates a write migration swapped entry      |
+-------------------------------+----------------------------------------------+

[1] https://lore.kernel.org/linux-mm/20181017020930.GN30832@redhat.com/
