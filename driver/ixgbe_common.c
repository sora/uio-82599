#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/mii.h>
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#include "ixgbe_type.h"
#include "ixgbe_common.h"
#include "ixgbe_phy.h"
#include "uio_ixgbe.h"

static s32 ixgbe_setup_fc(struct ixgbe_hw *hw);
static u32 ixgbe_pcie_timeout_poll(struct ixgbe_hw *hw);
static s32 ixgbe_read_eeprom_buffer_bit_bang(struct ixgbe_hw *hw, u16 offset,
                                             u16 words, u16 *data);
static s32 ixgbe_acquire_eeprom(struct ixgbe_hw *hw);
static s32 ixgbe_ready_eeprom(struct ixgbe_hw *hw);
static void ixgbe_release_eeprom(struct ixgbe_hw *hw);
static void ixgbe_standby_eeprom(struct ixgbe_hw *hw);
static void ixgbe_shift_out_eeprom_bits(struct ixgbe_hw *hw, u16 data,
                                        u16 count);
static u16 ixgbe_shift_in_eeprom_bits(struct ixgbe_hw *hw, u16 count);
static void ixgbe_raise_eeprom_clk(struct ixgbe_hw *hw, u32 *eec);
static void ixgbe_lower_eeprom_clk(struct ixgbe_hw *hw, u32 *eec);
static s32 ixgbe_get_eeprom_semaphore(struct ixgbe_hw *hw);
static void ixgbe_release_eeprom_semaphore(struct ixgbe_hw *hw);

s32 ixgbe_init_hw_generic(struct ixgbe_hw *hw){
        s32 status;

        /* Reset the hardware */
        status = hw->mac.ops.reset_hw(hw);

        if (status == 0) {
                /* Start the HW */
                status = hw->mac.ops.start_hw(hw);
        }

        return status;
}

s32 ixgbe_start_hw_generic(struct ixgbe_hw *hw)
{
        s32 ret_val;
        u32 ctrl_ext;

        /* Set the media type */
        hw->phy.media_type = hw->mac.ops.get_media_type(hw);

        /* PHY ops initialization must be done in reset_hw() */

        /* Clear the VLAN filter table */
        hw->mac.ops.clear_vfta(hw);

        /* Clear statistics registers */
        hw->mac.ops.clear_hw_cntrs(hw);

        /* Set No Snoop Disable */
        ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
        ctrl_ext |= IXGBE_CTRL_EXT_NS_DIS;
        IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);
        IXGBE_WRITE_FLUSH(hw);

        /* Setup flow control */
        ret_val = ixgbe_setup_fc(hw);
        if (ret_val != 0)
                goto out;

        /* Clear adapter stopped flag */
        hw->adapter_stopped = false;

out:
        return ret_val;
}

s32 ixgbe_start_hw_gen2(struct ixgbe_hw *hw)
{
        u32 i;
        u32 regval;

        /* Clear the rate limiters */
        for (i = 0; i < hw->mac.max_tx_queues; i++) {
                IXGBE_WRITE_REG(hw, IXGBE_RTTDQSEL, i);
                IXGBE_WRITE_REG(hw, IXGBE_RTTBCNRC, 0);
        }
        IXGBE_WRITE_FLUSH(hw);

        /* Disable relaxed ordering */
        for (i = 0; i < hw->mac.max_tx_queues; i++) {
                regval = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL_82599(i));
                regval &= ~IXGBE_DCA_TXCTRL_DESC_WRO_EN;
                IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL_82599(i), regval);
        }

        for (i = 0; i < hw->mac.max_rx_queues; i++) {
                regval = IXGBE_READ_REG(hw, IXGBE_DCA_RXCTRL(i));
                regval &= ~(IXGBE_DCA_RXCTRL_DATA_WRO_EN |
                            IXGBE_DCA_RXCTRL_HEAD_WRO_EN);
                IXGBE_WRITE_REG(hw, IXGBE_DCA_RXCTRL(i), regval);
        }

        return 0;
}

bool ixgbe_device_supports_autoneg_fc(struct ixgbe_hw *hw)
{
        bool supported = false;
        ixgbe_link_speed speed;
        bool link_up;

        switch (hw->phy.media_type) {
        case ixgbe_media_type_fiber:
                hw->mac.ops.check_link(hw, &speed, &link_up, false);
                /* if link is down, assume supported */
                if (link_up)
                        supported = speed == IXGBE_LINK_SPEED_1GB_FULL ?
                                true : false;
                else
                        supported = true;
                break;
        default:
                break;
        }

        return supported;
}

static s32 ixgbe_setup_fc(struct ixgbe_hw *hw)
{
        s32 ret_val = 0;
        u32 reg = 0;

        /*
         * Validate the requested mode.  Strict IEEE mode does not allow
         * ixgbe_fc_rx_pause because it will cause us to fail at UNH.
         */
        if (hw->fc.strict_ieee && hw->fc.requested_mode == ixgbe_fc_rx_pause) {
                ret_val = IXGBE_ERR_INVALID_LINK_SETTINGS;
                goto out;
        }

        /*
         * 10gig parts do not have a word in the EEPROM to determine the
         * default flow control setting, so we explicitly set it to full.
         */
        if (hw->fc.requested_mode == ixgbe_fc_default)
                hw->fc.requested_mode = ixgbe_fc_full;

        /*
         * Set up the 1G and 10G flow control advertisement registers so the
         * HW will be able to do fc autoneg once the cable is plugged in.  If
         * we link at 10G, the 1G advertisement is harmless and vice versa.
         */
        switch (hw->phy.media_type) {
        case ixgbe_media_type_fiber:
                reg = IXGBE_READ_REG(hw, IXGBE_PCS1GANA);

                break;
        default:
                break;
        }

        /*
         * The possible values of fc.requested_mode are:
         * 0: Flow control is completely disabled
         * 1: Rx flow control is enabled (we can receive pause frames,
         *    but not send pause frames).
         * 2: Tx flow control is enabled (we can send pause frames but
         *    we do not support receiving pause frames).
         * 3: Both Rx and Tx flow control (symmetric) are enabled.
         * other: Invalid.
         */
        switch (hw->fc.requested_mode) {
        case ixgbe_fc_none:
                /* Flow control completely disabled by software override. */
                reg &= ~(IXGBE_PCS1GANA_SYM_PAUSE | IXGBE_PCS1GANA_ASM_PAUSE);
                break;
        case ixgbe_fc_tx_pause:
                /*
                 * Tx Flow control is enabled, and Rx Flow control is
                 * disabled by software override.
                 */
                reg |= IXGBE_PCS1GANA_ASM_PAUSE;
                reg &= ~IXGBE_PCS1GANA_SYM_PAUSE;
                break;
        case ixgbe_fc_rx_pause:
                /*
                 * Rx Flow control is enabled and Tx Flow control is
                 * disabled by software override. Since there really
                 * isn't a way to advertise that we are capable of RX
                 * Pause ONLY, we will advertise that we support both
                 * symmetric and asymmetric Rx PAUSE, as such we fall
                 * through to the fc_full statement.  Later, we will
                 * disable the adapter's ability to send PAUSE frames.
                 */
        case ixgbe_fc_full:
                /* Flow control (both Rx and Tx) is enabled by SW override. */
                reg |= IXGBE_PCS1GANA_SYM_PAUSE | IXGBE_PCS1GANA_ASM_PAUSE;
                break;
        default:
                ret_val = IXGBE_ERR_CONFIG;
                goto out;
                break;
        }

        if (hw->mac.type < ixgbe_mac_X540) {
                /*
                 * Enable auto-negotiation between the MAC & PHY;
                 * the MAC will advertise clause 37 flow control.
                 */
                IXGBE_WRITE_REG(hw, IXGBE_PCS1GANA, reg);
                reg = IXGBE_READ_REG(hw, IXGBE_PCS1GLCTL);

                /* Disable AN timeout */
                if (hw->fc.strict_ieee)
                        reg &= ~IXGBE_PCS1GLCTL_AN_1G_TIMEOUT_EN;

                IXGBE_WRITE_REG(hw, IXGBE_PCS1GLCTL, reg);
        }

out:
        return ret_val;
}



s32 ixgbe_get_mac_addr_generic(struct ixgbe_hw *hw, u8 *mac_addr){
        u32 rar_high;
        u32 rar_low;
        u16 i;

        rar_high = IXGBE_READ_REG(hw, IXGBE_RAH(0));
        rar_low = IXGBE_READ_REG(hw, IXGBE_RAL(0));

        for (i = 0; i < 4; i++)
                mac_addr[i] = (u8)(rar_low >> (i*8));

        for (i = 0; i < 2; i++)
                mac_addr[i+4] = (u8)(rar_high >> (i*8));

        return 0;
}

void ixgbe_set_lan_id_multi_port_pcie(struct ixgbe_hw *hw)
{
        struct ixgbe_bus_info *bus = &hw->bus;
        u32 reg;

        reg = IXGBE_READ_REG(hw, IXGBE_STATUS);
        bus->func = (reg & IXGBE_STATUS_LAN_ID) >> IXGBE_STATUS_LAN_ID_SHIFT;
        bus->lan_id = bus->func;

        /* check for a port swap */
        reg = IXGBE_READ_REG(hw, IXGBE_FACTPS);
        if (reg & IXGBE_FACTPS_LFS)
                bus->func ^= 0x1;
}

s32 ixgbe_stop_adapter_generic(struct ixgbe_hw *hw){
        u32 reg_val;
        u16 i;

        /*
         * Set the adapter_stopped flag so other driver functions stop touching
         * the hardware
         */
        hw->adapter_stopped = true;

        /* Disable the receive unit */
        IXGBE_WRITE_REG(hw, IXGBE_RXCTRL, 0);

        /* Clear interrupt mask to stop interrupts from being generated */
        IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);

        /* Clear any pending interrupts, flush previous writes */
        IXGBE_READ_REG(hw, IXGBE_EICR);

        /* Disable the transmit unit.  Each queue must be disabled. */
        for (i = 0; i < hw->mac.max_tx_queues; i++)
                IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(i), IXGBE_TXDCTL_SWFLSH);

        /* Disable the receive unit by stopping each queue */
        for (i = 0; i < hw->mac.max_rx_queues; i++) {
                reg_val = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
                reg_val &= ~IXGBE_RXDCTL_ENABLE;
                reg_val |= IXGBE_RXDCTL_SWFLSH;
                IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), reg_val);
        }

        /* flush all queues disables */
        IXGBE_WRITE_FLUSH(hw);
        msleep(2);

        /*
         * Prevent the PCI-E bus from from hanging by disabling PCI-E master
         * access and verify no pending requests
         */
        return ixgbe_disable_pcie_master(hw);
}

s32 ixgbe_init_rx_addrs_generic(struct ixgbe_hw *hw){
        u32 i;
        u32 rar_entries = hw->mac.num_rar_entries;

        /*
         * If the current mac address is valid, assume it is a software override
         * to the permanent address.
         * Otherwise, use the permanent address from the eeprom.
         */
        if (ixgbe_validate_mac_addr(hw->mac.addr) ==
            IXGBE_ERR_INVALID_MAC_ADDR) {
                /* Get the MAC address from the RAR0 for later reference */
                hw->mac.ops.get_mac_addr(hw, hw->mac.addr);
        } else {
                /* Setup the receive address. */
                hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
        }

        /* Zero out the other receive addresses. */
        for (i = 1; i < rar_entries; i++) {
                IXGBE_WRITE_REG(hw, IXGBE_RAL(i), 0);
                IXGBE_WRITE_REG(hw, IXGBE_RAH(i), 0);
        }

        /* Clear the MTA */
        IXGBE_WRITE_REG(hw, IXGBE_MCSTCTRL, hw->mac.mc_filter_type);

        for (i = 0; i < hw->mac.mcft_size; i++)
                IXGBE_WRITE_REG(hw, IXGBE_MTA(i), 0);

        ixgbe_init_uta_tables_generic(hw);

        return 0;
}

s32 ixgbe_validate_mac_addr(u8 *mac_addr)
{
        s32 status = 0;

        /* Make sure it is not a multicast address */
        if (IXGBE_IS_MULTICAST(mac_addr)) {
                status = IXGBE_ERR_INVALID_MAC_ADDR;
        /* Not a broadcast address */
        } else if (IXGBE_IS_BROADCAST(mac_addr)) {
                status = IXGBE_ERR_INVALID_MAC_ADDR;
        /* Reject the zero address */
        } else if (mac_addr[0] == 0 && mac_addr[1] == 0 && mac_addr[2] == 0 &&
                   mac_addr[3] == 0 && mac_addr[4] == 0 && mac_addr[5] == 0) {
                status = IXGBE_ERR_INVALID_MAC_ADDR;
        }
        return status;
}

s32 ixgbe_validate_eeprom_checksum_generic(struct ixgbe_hw *hw, u16 *checksum_val){
        s32 status;
        u16 checksum;
        u16 read_checksum = 0;

        /*
         * Read the first word from the EEPROM. If this times out or fails, do
         * not continue or we could be in for a very long wait while every
         * EEPROM read fails
         */
        status = hw->eeprom.ops.read(hw, 0, &checksum);

        if (status == 0) {
                checksum = hw->eeprom.ops.calc_checksum(hw);

                hw->eeprom.ops.read(hw, IXGBE_EEPROM_CHECKSUM, &read_checksum);

                /*
                 * Verify read checksum from EEPROM is the same as
                 * calculated checksum
                 */
                if (read_checksum != checksum)
                        status = IXGBE_ERR_EEPROM_CHECKSUM;

                /* If the user cares, return the calculated checksum */
                if (checksum_val)
                        *checksum_val = checksum;
        }

        return status;
}

u8 ixgbe_calculate_checksum(u8 *buffer, u32 length)
{
        u32 i;
        u8 sum = 0;

        if (!buffer)
                return 0;
        for (i = 0; i < length; i++)
                sum += buffer[i];

        return (u8) (0 - sum);
}

s32 ixgbe_host_interface_command(struct ixgbe_hw *hw, u32 *buffer,
                                 u32 length)
{
        u32 hicr, i, bi;
        u32 hdr_size = sizeof(struct ixgbe_hic_hdr);
        u8 buf_len, dword_len;

        s32 ret_val = 0;

        if (length == 0 || length & 0x3 ||
            length > IXGBE_HI_MAX_BLOCK_BYTE_LENGTH) {
                ret_val = IXGBE_ERR_HOST_INTERFACE_COMMAND;
                goto out;
        }

        /* Check that the host interface is enabled. */
        hicr = IXGBE_READ_REG(hw, IXGBE_HICR);
        if ((hicr & IXGBE_HICR_EN) == 0) {
                ret_val = IXGBE_ERR_HOST_INTERFACE_COMMAND;
                goto out;
        }

        /* Calculate length in DWORDs */
        dword_len = length >> 2;

        /*
         * The device driver writes the relevant command block
         * into the ram area.
         */
        for (i = 0; i < dword_len; i++)
                IXGBE_WRITE_REG_ARRAY(hw, IXGBE_FLEX_MNG,
                                      i, IXGBE_CPU_TO_LE32(buffer[i]));

        /* Setting this bit tells the ARC that a new command is pending. */
        IXGBE_WRITE_REG(hw, IXGBE_HICR, hicr | IXGBE_HICR_C);

        for (i = 0; i < IXGBE_HI_COMMAND_TIMEOUT; i++) {
                hicr = IXGBE_READ_REG(hw, IXGBE_HICR);
                if (!(hicr & IXGBE_HICR_C))
                        break;
                msleep(1);
        }

        /* Check command successful completion. */
        if (i == IXGBE_HI_COMMAND_TIMEOUT ||
            (!(IXGBE_READ_REG(hw, IXGBE_HICR) & IXGBE_HICR_SV))) {
                ret_val = IXGBE_ERR_HOST_INTERFACE_COMMAND;
                goto out;
        }

        /* Calculate length in DWORDs */
        dword_len = hdr_size >> 2;

        /* first pull in the header so we know the buffer length */
        for (bi = 0; bi < dword_len; bi++) {
                buffer[bi] = IXGBE_READ_REG_ARRAY(hw, IXGBE_FLEX_MNG, bi);
                IXGBE_LE32_TO_CPUS(&buffer[bi]);
        }

        /* If there is any thing in data position pull it in */
        buf_len = ((struct ixgbe_hic_hdr *)buffer)->buf_len;
        if (buf_len == 0)
                goto out;

        if (length < (buf_len + hdr_size)) {
                ret_val = IXGBE_ERR_HOST_INTERFACE_COMMAND;
                goto out;
        }

        /* Calculate length in DWORDs, add 3 for odd lengths */
        dword_len = (buf_len + 3) >> 2;

        /* Pull in the rest of the buffer (bi is where we left off)*/
        for (; bi <= dword_len; bi++) {
                buffer[bi] = IXGBE_READ_REG_ARRAY(hw, IXGBE_FLEX_MNG, bi);
                IXGBE_LE32_TO_CPUS(&buffer[bi]);
        }

out:
        return ret_val;
}

s32 ixgbe_set_fw_drv_ver_generic(struct ixgbe_hw *hw, u8 maj, u8 min, u8 build, u8 sub){
        struct ixgbe_hic_drv_info fw_cmd;
        int i;
        s32 ret_val = 0;

        if (hw->mac.ops.acquire_swfw_sync(hw, IXGBE_GSSR_SW_MNG_SM)
            != 0) {
                ret_val = IXGBE_ERR_SWFW_SYNC;
                goto out;
        }

        fw_cmd.hdr.cmd = FW_CEM_CMD_DRIVER_INFO;
        fw_cmd.hdr.buf_len = FW_CEM_CMD_DRIVER_INFO_LEN;
        fw_cmd.hdr.cmd_or_resp.cmd_resv = FW_CEM_CMD_RESERVED;
        fw_cmd.port_num = (u8)hw->bus.func;
        fw_cmd.ver_maj = maj;
        fw_cmd.ver_min = min;
        fw_cmd.ver_build = build;
        fw_cmd.ver_sub = sub;
        fw_cmd.hdr.checksum = 0;
        fw_cmd.hdr.checksum = ixgbe_calculate_checksum((u8 *)&fw_cmd,
                                (FW_CEM_HDR_LEN + fw_cmd.hdr.buf_len));
        fw_cmd.pad = 0;
        fw_cmd.pad2 = 0;

        for (i = 0; i <= FW_CEM_MAX_RETRIES; i++) {
                ret_val = ixgbe_host_interface_command(hw, (u32 *)&fw_cmd,
                                                       sizeof(fw_cmd));
                if (ret_val != 0)
                        continue;

                if (fw_cmd.hdr.cmd_or_resp.ret_status ==
                    FW_CEM_RESP_STATUS_SUCCESS)
                        ret_val = 0;
                else
                        ret_val = IXGBE_ERR_HOST_INTERFACE_COMMAND;

                break;
        }

        hw->mac.ops.release_swfw_sync(hw, IXGBE_GSSR_SW_MNG_SM);
out:
        return ret_val;
}

s32 ixgbe_check_mac_link_generic(struct ixgbe_hw *hw, u32 *speed,
                                 bool *link_up, bool link_up_wait_to_complete){
        u32 links_reg, links_orig;
        u32 i;

        /* clear the old state */
        links_orig = IXGBE_READ_REG(hw, IXGBE_LINKS);

        links_reg = IXGBE_READ_REG(hw, IXGBE_LINKS);

        if (link_up_wait_to_complete) {
                for (i = 0; i < IXGBE_LINK_UP_TIME; i++) {
                        if (links_reg & IXGBE_LINKS_UP) {
                                *link_up = true;
                                break;
                        } else {
                                *link_up = false;
                        }
                        msleep(100);
                        links_reg = IXGBE_READ_REG(hw, IXGBE_LINKS);
                }
        } else {
                if (links_reg & IXGBE_LINKS_UP)
                        *link_up = true;
                else
                        *link_up = false;
        }

        if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
            IXGBE_LINKS_SPEED_10G_82599)
                *speed = IXGBE_LINK_SPEED_10GB_FULL;
        else if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
                 IXGBE_LINKS_SPEED_1G_82599)
                *speed = IXGBE_LINK_SPEED_1GB_FULL;
        else if ((links_reg & IXGBE_LINKS_SPEED_82599) ==
                 IXGBE_LINKS_SPEED_100_82599)
                *speed = IXGBE_LINK_SPEED_100_FULL;
        else
                *speed = IXGBE_LINK_SPEED_UNKNOWN;

        return 0;
}

s32 ixgbe_disable_pcie_master(struct ixgbe_hw *hw)
{
        s32 status = 0;
        u32 i, poll;
        u16 value;

        /* Always set this bit to ensure any future transactions are blocked */
        IXGBE_WRITE_REG(hw, IXGBE_CTRL, IXGBE_CTRL_GIO_DIS);

        /* Exit if master requests are blocked */
        if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO) ||
            IXGBE_REMOVED(hw->hw_addr))
                goto out;

        /* Poll for master request bit to clear */
        for (i = 0; i < IXGBE_PCI_MASTER_DISABLE_TIMEOUT; i++) {
                udelay(100);
                if (!(IXGBE_READ_REG(hw, IXGBE_STATUS) & IXGBE_STATUS_GIO))
                        goto out;
        }

        /*
         * Two consecutive resets are required via CTRL.RST per datasheet
         * 5.2.5.3.2 Master Disable.  We set a flag to inform the reset routine
         * of this need.  The first reset prevents new master requests from
         * being issued by our device.  We then must wait 1usec or more for any
         * remaining completions from the PCIe bus to trickle in, and then reset
         * again to clear out any effects they may have had on our device.
         */
        hw->mac.flags |= IXGBE_FLAGS_DOUBLE_RESET_REQUIRED;

        /*
         * Before proceeding, make sure that the PCIe block does not have
         * transactions pending.
         */
        poll = ixgbe_pcie_timeout_poll(hw);
        for (i = 0; i < poll; i++) {
                udelay(100);
                value = ixgbe_read_pci_cfg_word(hw, IXGBE_PCI_DEVICE_STATUS);
                if (IXGBE_REMOVED(hw->hw_addr))
                        goto out;
                if (!(value & IXGBE_PCI_DEVICE_STATUS_TRANSACTION_PENDING))
                        goto out;
        }

        status = IXGBE_ERR_MASTER_REQUESTS_PENDING;

out:
        return status;
}

u16 ixgbe_get_pcie_msix_count_generic(struct ixgbe_hw *hw){
        u16 msix_count = 1;
        u16 max_msix_count;
        u16 pcie_offset;

        switch (hw->mac.type) {
        case ixgbe_mac_82599EB:
                pcie_offset = IXGBE_PCIE_MSIX_82599_CAPS;
                max_msix_count = IXGBE_MAX_MSIX_VECTORS_82599;
                break;
        default:
                return msix_count;
        }

        msix_count = ixgbe_read_pci_cfg_word(hw, pcie_offset);
        if (IXGBE_REMOVED(hw->hw_addr))
                msix_count = 0;
        msix_count &= IXGBE_PCIE_MSIX_TBL_SZ_MASK;

        /* MSI-X count is zero-based in HW */
        msix_count++;

        if (msix_count > max_msix_count)
                msix_count = max_msix_count;

        return msix_count;
}

void ixgbe_clear_tx_pending(struct ixgbe_hw *hw)
{
        u32 gcr_ext, hlreg0;

        /*
         * If double reset is not requested then all transactions should
         * already be clear and as such there is no work to do
         */
        if (!(hw->mac.flags & IXGBE_FLAGS_DOUBLE_RESET_REQUIRED))
                return;

        /*
         * Set loopback enable to prevent any transmits from being sent
         * should the link come up.  This assumes that the RXCTRL.RXEN bit
         * has already been cleared.
         */
        hlreg0 = IXGBE_READ_REG(hw, IXGBE_HLREG0);
        IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0 | IXGBE_HLREG0_LPBK);

        /* initiate cleaning flow for buffers in the PCIe transaction layer */
        gcr_ext = IXGBE_READ_REG(hw, IXGBE_GCR_EXT);
        IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT,
                        gcr_ext | IXGBE_GCR_EXT_BUFFERS_CLEAR);

        /* Flush all writes and allow 20usec for all transactions to clear */
        IXGBE_WRITE_FLUSH(hw);
        udelay(20);

        /* restore previous register values */
        IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT, gcr_ext);
        IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0);
}

s32 ixgbe_read_eerd_generic(struct ixgbe_hw *hw, u16 offset, u16 *data)
{
        return ixgbe_read_eerd_buffer_generic(hw, offset, 1, data);
}

s32 ixgbe_read_eerd_buffer_generic(struct ixgbe_hw *hw, u16 offset,
                                   u16 words, u16 *data)
{
        u32 eerd;
        s32 status = 0;
        u32 i;

        hw->eeprom.ops.init_params(hw);

        if (words == 0) {
                status = IXGBE_ERR_INVALID_ARGUMENT;
                goto out;
        }

        if (offset >= hw->eeprom.word_size) {
                status = IXGBE_ERR_EEPROM;
                goto out;
        }

        for (i = 0; i < words; i++) {
                eerd = ((offset + i) << IXGBE_EEPROM_RW_ADDR_SHIFT) |
                       IXGBE_EEPROM_RW_REG_START;

                IXGBE_WRITE_REG(hw, IXGBE_EERD, eerd);
                status = ixgbe_poll_eerd_eewr_done(hw, IXGBE_NVM_POLL_READ);

                if (status == 0) {
                        data[i] = (IXGBE_READ_REG(hw, IXGBE_EERD) >>
                                   IXGBE_EEPROM_RW_REG_DATA);
                } else {
                        goto out;
                }
        }
out:
        return status;
}

s32 ixgbe_read_eeprom_bit_bang_generic(struct ixgbe_hw *hw, u16 offset,
                                       u16 *data)
{
        s32 status;

        hw->eeprom.ops.init_params(hw);

        if (offset >= hw->eeprom.word_size) {
                status = IXGBE_ERR_EEPROM;
                goto out;
        }

        status = ixgbe_read_eeprom_buffer_bit_bang(hw, offset, 1, data);

out:
        return status;
}

static s32 ixgbe_read_eeprom_buffer_bit_bang(struct ixgbe_hw *hw, u16 offset,
                                             u16 words, u16 *data)
{
        s32 status;
        u16 word_in;
        u8 read_opcode = IXGBE_EEPROM_READ_OPCODE_SPI;
        u16 i;

        /* Prepare the EEPROM for reading  */
        status = ixgbe_acquire_eeprom(hw);

        if (status == 0) {
                if (ixgbe_ready_eeprom(hw) != 0) {
                        ixgbe_release_eeprom(hw);
                        status = IXGBE_ERR_EEPROM;
                }
        }

        if (status == 0) {
                for (i = 0; i < words; i++) {
                        ixgbe_standby_eeprom(hw);
                        /*
                         * Some SPI eeproms use the 8th address bit embedded
                         * in the opcode
                         */
                        if ((hw->eeprom.address_bits == 8) &&
                            ((offset + i) >= 128))
                                read_opcode |= IXGBE_EEPROM_A8_OPCODE_SPI;

                        /* Send the READ command (opcode + addr) */
                        ixgbe_shift_out_eeprom_bits(hw, read_opcode,
                                                    IXGBE_EEPROM_OPCODE_BITS);
                        ixgbe_shift_out_eeprom_bits(hw, (u16)((offset + i) * 2),
                                                    hw->eeprom.address_bits);

                        /* Read the data. */
                        word_in = ixgbe_shift_in_eeprom_bits(hw, 16);
                        data[i] = (word_in >> 8) | (word_in << 8);
                }

                /* End this read operation */
                ixgbe_release_eeprom(hw);
        }

        return status;
}

s32 ixgbe_init_uta_tables_generic(struct ixgbe_hw *hw)
{
        int i;

        for (i = 0; i < 128; i++)
                IXGBE_WRITE_REG(hw, IXGBE_UTA(i), 0);

        return 0;
}

static u32 ixgbe_pcie_timeout_poll(struct ixgbe_hw *hw)
{
        s16 devctl2;
        u32 pollcnt;

        devctl2 = ixgbe_read_pci_cfg_word(hw, IXGBE_PCI_DEVICE_CONTROL2);
        devctl2 &= IXGBE_PCIDEVCTRL2_TIMEO_MASK;

        switch (devctl2) {
        case IXGBE_PCIDEVCTRL2_65_130ms:
                pollcnt = 1300;         /* 130 millisec */
                break;
        case IXGBE_PCIDEVCTRL2_260_520ms:
                pollcnt = 5200;         /* 520 millisec */
                break;
        case IXGBE_PCIDEVCTRL2_1_2s:
                pollcnt = 20000;        /* 2 sec */
                break;
        case IXGBE_PCIDEVCTRL2_4_8s:
                pollcnt = 80000;        /* 8 sec */
                break;
        case IXGBE_PCIDEVCTRL2_17_34s:
                pollcnt = 34000;        /* 34 sec */
                break;
        case IXGBE_PCIDEVCTRL2_50_100us:        /* 100 microsecs */
        case IXGBE_PCIDEVCTRL2_1_2ms:           /* 2 millisecs */
        case IXGBE_PCIDEVCTRL2_16_32ms:         /* 32 millisec */
        case IXGBE_PCIDEVCTRL2_16_32ms_def:     /* 32 millisec default */
        default:
                pollcnt = 800;          /* 80 millisec minimum */
                break;
        }

        /* add 10% to spec maximum */
        return (pollcnt * 11) / 10;
}

s32 ixgbe_poll_eerd_eewr_done(struct ixgbe_hw *hw, u32 ee_reg)
{
        u32 i;
        u32 reg;
        s32 status = IXGBE_ERR_EEPROM;

        for (i = 0; i < IXGBE_EERD_EEWR_ATTEMPTS; i++) {
                if (ee_reg == IXGBE_NVM_POLL_READ)
                        reg = IXGBE_READ_REG(hw, IXGBE_EERD);
                else
                        reg = IXGBE_READ_REG(hw, IXGBE_EEWR);

                if (reg & IXGBE_EEPROM_RW_REG_DONE) {
                        status = 0;
                        break;
                }
                udelay(5);
        }

        return status;
}

static s32 ixgbe_acquire_eeprom(struct ixgbe_hw *hw)
{
        s32 status = 0;
        u32 eec;
        u32 i;

        if (hw->mac.ops.acquire_swfw_sync(hw, IXGBE_GSSR_EEP_SM)
            != 0)
                status = IXGBE_ERR_SWFW_SYNC;

        if (status == 0) {
                eec = IXGBE_READ_REG(hw, IXGBE_EEC);

                /* Request EEPROM Access */
                eec |= IXGBE_EEC_REQ;
                IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);

                for (i = 0; i < IXGBE_EEPROM_GRANT_ATTEMPTS; i++) {
                        eec = IXGBE_READ_REG(hw, IXGBE_EEC);
                        if (eec & IXGBE_EEC_GNT)
                                break;
                        udelay(5);
                }

                /* Release if grant not acquired */
                if (!(eec & IXGBE_EEC_GNT)) {
                        eec &= ~IXGBE_EEC_REQ;
                        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);

                        hw->mac.ops.release_swfw_sync(hw, IXGBE_GSSR_EEP_SM);
                        status = IXGBE_ERR_EEPROM;
                }

                /* Setup EEPROM for Read/Write */
                if (status == 0) {
                        /* Clear CS and SK */
                        eec &= ~(IXGBE_EEC_CS | IXGBE_EEC_SK);
                        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
                        IXGBE_WRITE_FLUSH(hw);
                        udelay(1);
                }
        }
        return status;
}

static s32 ixgbe_ready_eeprom(struct ixgbe_hw *hw)
{
        s32 status = 0;
        u16 i;
        u8 spi_stat_reg;

        /*
         * Read "Status Register" repeatedly until the LSB is cleared.  The
         * EEPROM will signal that the command has been completed by clearing
         * bit 0 of the internal status register.  If it's not cleared within
         * 5 milliseconds, then error out.
         */
        for (i = 0; i < IXGBE_EEPROM_MAX_RETRY_SPI; i += 5) {
                ixgbe_shift_out_eeprom_bits(hw, IXGBE_EEPROM_RDSR_OPCODE_SPI,
                                            IXGBE_EEPROM_OPCODE_BITS);
                spi_stat_reg = (u8)ixgbe_shift_in_eeprom_bits(hw, 8);
                if (!(spi_stat_reg & IXGBE_EEPROM_STATUS_RDY_SPI))
                        break;

                udelay(5);
                ixgbe_standby_eeprom(hw);
        };

        /*
         * On some parts, SPI write time could vary from 0-20mSec on 3.3V
         * devices (and only 0-5mSec on 5V devices)
         */
        if (i >= IXGBE_EEPROM_MAX_RETRY_SPI) {
                status = IXGBE_ERR_EEPROM;
        }

        return status;
}

static void ixgbe_release_eeprom(struct ixgbe_hw *hw)
{
        u32 eec;

        eec = IXGBE_READ_REG(hw, IXGBE_EEC);

        eec |= IXGBE_EEC_CS;  /* Pull CS high */
        eec &= ~IXGBE_EEC_SK; /* Lower SCK */

        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
        IXGBE_WRITE_FLUSH(hw);

        udelay(1);

        /* Stop requesting EEPROM access */
        eec &= ~IXGBE_EEC_REQ;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);

        hw->mac.ops.release_swfw_sync(hw, IXGBE_GSSR_EEP_SM);

        /* Delay before attempt to obtain semaphore again to allow FW access */
        msleep(hw->eeprom.semaphore_delay);
}

static void ixgbe_standby_eeprom(struct ixgbe_hw *hw)
{
        u32 eec;

        eec = IXGBE_READ_REG(hw, IXGBE_EEC);

        /* Toggle CS to flush commands */
        eec |= IXGBE_EEC_CS;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
        IXGBE_WRITE_FLUSH(hw);
        udelay(1);
        eec &= ~IXGBE_EEC_CS;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
        IXGBE_WRITE_FLUSH(hw);
        udelay(1);
}

static void ixgbe_shift_out_eeprom_bits(struct ixgbe_hw *hw, u16 data,
                                        u16 count)
{
        u32 eec;
        u32 mask;
        u32 i;

        eec = IXGBE_READ_REG(hw, IXGBE_EEC);

        /*
         * Mask is used to shift "count" bits of "data" out to the EEPROM
         * one bit at a time.  Determine the starting bit based on count
         */
        mask = 0x01 << (count - 1);

        for (i = 0; i < count; i++) {
                /*
                 * A "1" is shifted out to the EEPROM by setting bit "DI" to a
                 * "1", and then raising and then lowering the clock (the SK
                 * bit controls the clock input to the EEPROM).  A "0" is
                 * shifted out to the EEPROM by setting "DI" to "0" and then
                 * raising and then lowering the clock.
                 */
                if (data & mask)
                        eec |= IXGBE_EEC_DI;
                else
                        eec &= ~IXGBE_EEC_DI;

                IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
                IXGBE_WRITE_FLUSH(hw);

                udelay(1);

                ixgbe_raise_eeprom_clk(hw, &eec);
                ixgbe_lower_eeprom_clk(hw, &eec);

                /*
                 * Shift mask to signify next bit of data to shift in to the
                 * EEPROM
                 */
                mask = mask >> 1;
        };

        /* We leave the "DI" bit set to "0" when we leave this routine. */
        eec &= ~IXGBE_EEC_DI;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, eec);
        IXGBE_WRITE_FLUSH(hw);
}

static u16 ixgbe_shift_in_eeprom_bits(struct ixgbe_hw *hw, u16 count)
{
        u32 eec;
        u32 i;
        u16 data = 0;

        /*
         * In order to read a register from the EEPROM, we need to shift
         * 'count' bits in from the EEPROM. Bits are "shifted in" by raising
         * the clock input to the EEPROM (setting the SK bit), and then reading
         * the value of the "DO" bit.  During this "shifting in" process the
         * "DI" bit should always be clear.
         */
        eec = IXGBE_READ_REG(hw, IXGBE_EEC);

        eec &= ~(IXGBE_EEC_DO | IXGBE_EEC_DI);

        for (i = 0; i < count; i++) {
                data = data << 1;
                ixgbe_raise_eeprom_clk(hw, &eec);

                eec = IXGBE_READ_REG(hw, IXGBE_EEC);

                eec &= ~(IXGBE_EEC_DI);
                if (eec & IXGBE_EEC_DO)
                        data |= 1;

                ixgbe_lower_eeprom_clk(hw, &eec);
        }

        return data;
}

static void ixgbe_raise_eeprom_clk(struct ixgbe_hw *hw, u32 *eec)
{
        /*
         * Raise the clock input to the EEPROM
         * (setting the SK bit), then delay
         */
        *eec = *eec | IXGBE_EEC_SK;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, *eec);
        IXGBE_WRITE_FLUSH(hw);
        udelay(1);
}

static void ixgbe_lower_eeprom_clk(struct ixgbe_hw *hw, u32 *eec)
{
        /*
         * Lower the clock input to the EEPROM (clearing the SK bit), then
         * delay
         */
        *eec = *eec & ~IXGBE_EEC_SK;
        IXGBE_WRITE_REG(hw, IXGBE_EEC, *eec);
        IXGBE_WRITE_FLUSH(hw);
        udelay(1);
}

s32 ixgbe_read_pba_string_generic(struct ixgbe_hw *hw, u8 *pba_num,
                                  u32 pba_num_size)
{
        s32 ret_val;
        u16 data;
        u16 pba_ptr;
        u16 offset;
        u16 length;

        if (pba_num == NULL) {
                return IXGBE_ERR_INVALID_ARGUMENT;
        }

        ret_val = hw->eeprom.ops.read(hw, IXGBE_PBANUM0_PTR, &data);
        if (ret_val) {
                return ret_val;
        }

        ret_val = hw->eeprom.ops.read(hw, IXGBE_PBANUM1_PTR, &pba_ptr);
        if (ret_val) {
                return ret_val;
        }

        /*
         * if data is not ptr guard the PBA must be in legacy format which
         * means pba_ptr is actually our second data word for the PBA number
         * and we can decode it into an ascii string
         */
        if (data != IXGBE_PBANUM_PTR_GUARD) {
                /* we will need 11 characters to store the PBA */
                if (pba_num_size < 11) {
                        return IXGBE_ERR_NO_SPACE;
                }

                /* extract hex string from data and pba_ptr */
                pba_num[0] = (data >> 12) & 0xF;
                pba_num[1] = (data >> 8) & 0xF;
                pba_num[2] = (data >> 4) & 0xF;
                pba_num[3] = data & 0xF;
                pba_num[4] = (pba_ptr >> 12) & 0xF;
                pba_num[5] = (pba_ptr >> 8) & 0xF;
                pba_num[6] = '-';
                pba_num[7] = 0;
                pba_num[8] = (pba_ptr >> 4) & 0xF;
                pba_num[9] = pba_ptr & 0xF;

                /* put a null character on the end of our string */
                pba_num[10] = '\0';

                /* switch all the data but the '-' to hex char */
                for (offset = 0; offset < 10; offset++) {
                        if (pba_num[offset] < 0xA)
                                pba_num[offset] += '0';
                        else if (pba_num[offset] < 0x10)
                                pba_num[offset] += 'A' - 0xA;
                }

                return 0;
        }

        ret_val = hw->eeprom.ops.read(hw, pba_ptr, &length);
        if (ret_val) {
                return ret_val;
        }

        if (length == 0xFFFF || length == 0) {
                return IXGBE_ERR_PBA_SECTION;
        }

        /* check if pba_num buffer is big enough */
        if (pba_num_size  < (((u32)length * 2) - 1)) {
                return IXGBE_ERR_NO_SPACE;
        }

        /* trim pba length from start of string */
        pba_ptr++;
        length--;

        for (offset = 0; offset < length; offset++) {
                ret_val = hw->eeprom.ops.read(hw, pba_ptr + offset, &data);
                if (ret_val) {
                        return ret_val;
                }
                pba_num[offset * 2] = (u8)(data >> 8);
                pba_num[(offset * 2) + 1] = (u8)(data & 0xFF);
        }
        pba_num[offset * 2] = '\0';

        return 0;
}

s32 ixgbe_set_rar_generic(struct ixgbe_hw *hw, u32 index, u8 *addr, u32 vmdq,
                          u32 enable_addr)
{
        u32 rar_low, rar_high;
        u32 rar_entries = hw->mac.num_rar_entries;

        /* Make sure we are using a valid rar index range */
        if (index >= rar_entries) {
                return IXGBE_ERR_INVALID_ARGUMENT;
        }

        /*
         * HW expects these in little endian so we reverse the byte
         * order from network order (big endian) to little endian
         */
        rar_low = ((u32)addr[0] |
                   ((u32)addr[1] << 8) |
                   ((u32)addr[2] << 16) |
                   ((u32)addr[3] << 24));
        /*
         * Some parts put the VMDq setting in the extra RAH bits,
         * so save everything except the lower 16 bits that hold part
         * of the address and the address valid bit.
         */
        rar_high = IXGBE_READ_REG(hw, IXGBE_RAH(index));
        rar_high &= ~(0x0000FFFF | IXGBE_RAH_AV);
        rar_high |= ((u32)addr[4] | ((u32)addr[5] << 8));

        if (enable_addr != 0)
                rar_high |= IXGBE_RAH_AV;

        IXGBE_WRITE_REG(hw, IXGBE_RAL(index), rar_low);
        IXGBE_WRITE_REG(hw, IXGBE_RAH(index), rar_high);

        return 0;
}

s32 ixgbe_acquire_swfw_sync(struct ixgbe_hw *hw, u16 mask)
{
        u32 gssr = 0;
        u32 swmask = mask;
        u32 fwmask = mask << 5;
        u32 timeout = 200;
        u32 i;

        for (i = 0; i < timeout; i++) {
                /*
                 * SW NVM semaphore bit is used for access to all
                 * SW_FW_SYNC bits (not just NVM)
                 */
                if (ixgbe_get_eeprom_semaphore(hw))
                        return IXGBE_ERR_SWFW_SYNC;

                gssr = IXGBE_READ_REG(hw, IXGBE_GSSR);
                if (!(gssr & (fwmask | swmask))) {
                        gssr |= swmask;
                        IXGBE_WRITE_REG(hw, IXGBE_GSSR, gssr);
                        ixgbe_release_eeprom_semaphore(hw);
                        return 0;
                } else {
                        /* Resource is currently in use by FW or SW */
                        ixgbe_release_eeprom_semaphore(hw);
                        msleep(5);
                }
        }

        /* If time expired clear the bits holding the lock and retry */
        if (gssr & (fwmask | swmask))
                ixgbe_release_swfw_sync(hw, gssr & (fwmask | swmask));

        msleep(5);
        return IXGBE_ERR_SWFW_SYNC;
}

void ixgbe_release_swfw_sync(struct ixgbe_hw *hw, u16 mask)
{
        u32 gssr;
        u32 swmask = mask;

        ixgbe_get_eeprom_semaphore(hw);

        gssr = IXGBE_READ_REG(hw, IXGBE_GSSR);
        gssr &= ~swmask;
        IXGBE_WRITE_REG(hw, IXGBE_GSSR, gssr);

        ixgbe_release_eeprom_semaphore(hw);
}

s32 ixgbe_clear_vfta_generic(struct ixgbe_hw *hw)
{
        u32 offset;

        for (offset = 0; offset < hw->mac.vft_size; offset++)
                IXGBE_WRITE_REG(hw, IXGBE_VFTA(offset), 0);

        for (offset = 0; offset < IXGBE_VLVF_ENTRIES; offset++) {
                IXGBE_WRITE_REG(hw, IXGBE_VLVF(offset), 0);
                IXGBE_WRITE_REG(hw, IXGBE_VLVFB(offset * 2), 0);
                IXGBE_WRITE_REG(hw, IXGBE_VLVFB((offset * 2) + 1), 0);
        }

        return 0;
}

s32 ixgbe_clear_hw_cntrs_generic(struct ixgbe_hw *hw)
{
        u16 i = 0;

        IXGBE_READ_REG(hw, IXGBE_CRCERRS);
        IXGBE_READ_REG(hw, IXGBE_ILLERRC);
        IXGBE_READ_REG(hw, IXGBE_ERRBC);
        IXGBE_READ_REG(hw, IXGBE_MSPDC);

        for (i = 0; i < 8; i++)
                IXGBE_READ_REG(hw, IXGBE_MPC(i));

        IXGBE_READ_REG(hw, IXGBE_MLFC);
        IXGBE_READ_REG(hw, IXGBE_MRFC);
        IXGBE_READ_REG(hw, IXGBE_RLEC);
        IXGBE_READ_REG(hw, IXGBE_LXONTXC);
        IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
        IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
        IXGBE_READ_REG(hw, IXGBE_LXOFFRXCNT);

        for (i = 0; i < 8; i++) {
                IXGBE_READ_REG(hw, IXGBE_PXONTXC(i));
                IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(i));
                IXGBE_READ_REG(hw, IXGBE_PXONRXCNT(i));
                IXGBE_READ_REG(hw, IXGBE_PXOFFRXCNT(i));
        }

        for (i = 0; i < 8; i++)
        	IXGBE_READ_REG(hw, IXGBE_PXON2OFFCNT(i));

        IXGBE_READ_REG(hw, IXGBE_PRC64);
        IXGBE_READ_REG(hw, IXGBE_PRC127);
        IXGBE_READ_REG(hw, IXGBE_PRC255);
        IXGBE_READ_REG(hw, IXGBE_PRC511);
        IXGBE_READ_REG(hw, IXGBE_PRC1023);
        IXGBE_READ_REG(hw, IXGBE_PRC1522);
        IXGBE_READ_REG(hw, IXGBE_GPRC);
        IXGBE_READ_REG(hw, IXGBE_BPRC);
        IXGBE_READ_REG(hw, IXGBE_MPRC);
        IXGBE_READ_REG(hw, IXGBE_GPTC);
        IXGBE_READ_REG(hw, IXGBE_GORCL);
        IXGBE_READ_REG(hw, IXGBE_GORCH);
        IXGBE_READ_REG(hw, IXGBE_GOTCL);
        IXGBE_READ_REG(hw, IXGBE_GOTCH);
        IXGBE_READ_REG(hw, IXGBE_RUC);
        IXGBE_READ_REG(hw, IXGBE_RFC);
        IXGBE_READ_REG(hw, IXGBE_ROC);
        IXGBE_READ_REG(hw, IXGBE_RJC);
        IXGBE_READ_REG(hw, IXGBE_MNGPRC);
        IXGBE_READ_REG(hw, IXGBE_MNGPDC);
        IXGBE_READ_REG(hw, IXGBE_MNGPTC);
        IXGBE_READ_REG(hw, IXGBE_TORL);
        IXGBE_READ_REG(hw, IXGBE_TORH);
        IXGBE_READ_REG(hw, IXGBE_TPR);
        IXGBE_READ_REG(hw, IXGBE_TPT);
        IXGBE_READ_REG(hw, IXGBE_PTC64);
        IXGBE_READ_REG(hw, IXGBE_PTC127);
        IXGBE_READ_REG(hw, IXGBE_PTC255);
        IXGBE_READ_REG(hw, IXGBE_PTC511);
        IXGBE_READ_REG(hw, IXGBE_PTC1023);
        IXGBE_READ_REG(hw, IXGBE_PTC1522);
        IXGBE_READ_REG(hw, IXGBE_MPTC);
        IXGBE_READ_REG(hw, IXGBE_BPTC);
        for (i = 0; i < 16; i++) {
                IXGBE_READ_REG(hw, IXGBE_QPRC(i));
                IXGBE_READ_REG(hw, IXGBE_QPTC(i));
                IXGBE_READ_REG(hw, IXGBE_QBRC_L(i));
                IXGBE_READ_REG(hw, IXGBE_QBRC_H(i));
                IXGBE_READ_REG(hw, IXGBE_QBTC_L(i));
                IXGBE_READ_REG(hw, IXGBE_QBTC_H(i));
                IXGBE_READ_REG(hw, IXGBE_QPRDC(i));
        }

        return 0;
}

static s32 ixgbe_get_eeprom_semaphore(struct ixgbe_hw *hw)
{
        s32 status = IXGBE_ERR_EEPROM;
        u32 timeout = 2000;
        u32 i;
        u32 swsm;

        /* Get SMBI software semaphore between device drivers first */
        for (i = 0; i < timeout; i++) {
                /*
                 * If the SMBI bit is 0 when we read it, then the bit will be
                 * set and we have the semaphore
                 */
                swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);
                if (!(swsm & IXGBE_SWSM_SMBI)) {
                        status = 0;
                        break;
                }
                udelay(50);
        }

        if (i == timeout) {
                /*
                 * this release is particularly important because our attempts
                 * above to get the semaphore may have succeeded, and if there
                 * was a timeout, we should unconditionally clear the semaphore
                 * bits to free the driver to make progress
                 */
                ixgbe_release_eeprom_semaphore(hw);

                udelay(50);
                /*
                 * one last try
                 * If the SMBI bit is 0 when we read it, then the bit will be
                 * set and we have the semaphore
                 */
                swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);
                if (!(swsm & IXGBE_SWSM_SMBI))
                        status = 0;
        }

        /* Now get the semaphore between SW/FW through the SWESMBI bit */
        if (status == 0) {
                for (i = 0; i < timeout; i++) {
                        swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);

                        /* Set the SW EEPROM semaphore bit to request access */
                        swsm |= IXGBE_SWSM_SWESMBI;
                        IXGBE_WRITE_REG(hw, IXGBE_SWSM, swsm);

                        /*
                         * If we set the bit successfully then we got the
                         * semaphore.
                         */
                        swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);
                        if (swsm & IXGBE_SWSM_SWESMBI)
                                break;

                        udelay(50);
                }

                /*
                 * Release semaphores and return error if SW EEPROM semaphore
                 * was not granted because we don't have access to the EEPROM
                 */
                if (i >= timeout) {
                        ixgbe_release_eeprom_semaphore(hw);
                        status = IXGBE_ERR_EEPROM;
                }
        }

        return status;
}

static void ixgbe_release_eeprom_semaphore(struct ixgbe_hw *hw)
{
        u32 swsm;

        swsm = IXGBE_READ_REG(hw, IXGBE_SWSM);

        /* Release both semaphores by writing 0 to the bits SWESMBI and SMBI */
        swsm &= ~(IXGBE_SWSM_SWESMBI | IXGBE_SWSM_SMBI);
        IXGBE_WRITE_REG(hw, IXGBE_SWSM, swsm);
        IXGBE_WRITE_FLUSH(hw);
}

s32 ixgbe_init_eeprom_params_generic(struct ixgbe_hw *hw)
{
        struct ixgbe_eeprom_info *eeprom = &hw->eeprom;
        u32 eec;
        u16 eeprom_size;

        if (eeprom->type == ixgbe_eeprom_uninitialized) {
                eeprom->type = ixgbe_eeprom_none;
                /* Set default semaphore delay to 10ms which is a well
                 * tested value */
                eeprom->semaphore_delay = 10;
                /* Clear EEPROM page size, it will be initialized as needed */
                eeprom->word_page_size = 0;

                /*
                 * Check for EEPROM present first.
                 * If not present leave as none
                 */
                eec = IXGBE_READ_REG(hw, IXGBE_EEC);
                if (eec & IXGBE_EEC_PRES) {
                        eeprom->type = ixgbe_eeprom_spi;

                        /*
                         * SPI EEPROM is assumed here.  This code would need to
                         * change if a future EEPROM is not SPI.
                         */
                        eeprom_size = (u16)((eec & IXGBE_EEC_SIZE) >>
                                            IXGBE_EEC_SIZE_SHIFT);
                        eeprom->word_size = 1 << (eeprom_size +
                                             IXGBE_EEPROM_WORD_SIZE_SHIFT);
                }

                if (eec & IXGBE_EEC_ADDR_SIZE)
                        eeprom->address_bits = 16;
                else
                        eeprom->address_bits = 8;
        }

        return 0;
}

u16 ixgbe_calc_eeprom_checksum_generic(struct ixgbe_hw *hw)
{
        u16 i;
        u16 j;
        u16 checksum = 0;
        u16 length = 0;
        u16 pointer = 0;
        u16 word = 0;

        /* Include 0x0-0x3F in the checksum */
        for (i = 0; i < IXGBE_EEPROM_CHECKSUM; i++) {
                if (hw->eeprom.ops.read(hw, i, &word) != 0) {
                        break;
                }
                checksum += word;
        }

        /* Include all data from pointers except for the fw pointer */
        for (i = IXGBE_PCIE_ANALOG_PTR; i < IXGBE_FW_PTR; i++) {
                hw->eeprom.ops.read(hw, i, &pointer);

                /* Make sure the pointer seems valid */
                if (pointer != 0xFFFF && pointer != 0) {
                        hw->eeprom.ops.read(hw, pointer, &length);

                        if (length != 0xFFFF && length != 0) {
                                for (j = pointer+1; j <= pointer+length; j++) {
                                        hw->eeprom.ops.read(hw, j, &word);
                                        checksum += word;
                                }
                        }
                }
        }

        checksum = (u16)IXGBE_EEPROM_SUM - checksum;

        return checksum;
}
