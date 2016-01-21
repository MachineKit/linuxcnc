
//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//


#define HM2_SOC_VERSION "0.7"

#define HM2_LLIO_NAME "hm2_cvsoc"


#define HM2_SOC_MAX_BOARDS  1


// 
// PCI Device IDs and SubSystem Device IDs
//


#define HM2_PCI_SSDEV_5I21     (0x3312)

#define HM2_PCI_SSDEV_5I22_10  (0x3314)
#define HM2_PCI_SSDEV_5I22_15  (0x3313)

#define HM2_PCI_SSDEV_5I23     (0x3315)

#define HM2_PCI_SSDEV_5I24     (0x5124)
#define HM2_PCI_SSDEV_5I25     (0x5125)
#define HM2_PCI_SSDEV_6I25     (0x6125)


//
// PLX 9054 (5i22, 5i23, 4i68, 4i69)
//
// Note: also used for the PLX 9056 (3x20)
//

/* I/O register indices.
*/
#define CTRL_STAT_OFFSET_5I22	0x006C /* 5I22 32-bit control/status register. */

/* bit number in 9054 GPIO register */
/* yes, the direction control bits are not in the same order as the I/O bits */
#define DONE_MASK_5I22			(1<<17)	/* GPI */
#define _PROGRAM_MASK_5I22		(1<<16)	/* GPO, active low */
#define DONE_ENABLE_5I22		(1<<18) /* GPI direction control, 1=input */
#define _PROG_ENABLE_5I22		(1<<19) /* GPO direction control, 1=output */

/* how long should we wait for DONE when programming 9054-based cards */
#define DONE_WAIT_5I22			20000


typedef struct {
    struct pci_dev *dev;
    void __iomem *base;
    int len;
    unsigned long ctrl_base_addr;
    unsigned long data_base_addr;
    hm2_lowlevel_io_t llio;
} hm2_pci_t;

