/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef _PCI_EP_DEF_H
#define _PCI_EP_DEF_H

#define PCI_CONFIG_BAR_ID	0

#define PCI_BAR0_MNG_CH_BASE	(0)
#define PCI_BAR0_MNG_CH_SIZE	(0x1000)

#define PCI_BAR0_MQA_QNPT_BASE	(PCI_BAR0_MNG_CH_BASE + PCI_BAR0_MNG_CH_SIZE)
#define PCI_BAR0_MQA_QNPT_SIZE	(0x1000)

#define PCI_BAR0_MQA_QNCT_BASE	(PCI_BAR0_MQA_QNPT_BASE + PCI_BAR0_MQA_QNPT_SIZE)
#define PCI_BAR0_MQA_QNCT_SIZE	(0x1000)

#define PCI_BAR0_MSI_X_TBL_BASE (0x100000)
#define PCI_BAR0_MSI_X_TBL_SIZE (0x100000)

#define PCI_BAR0_MSI_X_PBA_BASE (PCI_BAR0_MSI_X_TBL_BASE + PCI_BAR0_MSI_X_TBL_SIZE)
#define PCI_BAR0_MSI_X_PBA_SIZE (0x100000)

#endif /* _PCI_EP_DEF_H */
