// SPDX-License-Identifier: GPL-2.0
/*
 * MDIO bus driver for the Xilinx Axi Ethernet device
 *
 * Copyright (c) 2009 Secret Lab Technologies, Ltd.
 * Copyright (c) 2010 - 2011 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2010 - 2011 PetaLogix
 * Copyright (c) 2019 SED Systems, a division of Calian Ltd.
 * Copyright (c) 2010 - 2012 Xilinx, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/jiffies.h>
#include <linux/iopoll.h>

#include "xilinx_axienet.h"

#define MAX_MDIO_FREQ		2000000 /* 2.0 MHz */
#define DEFAULT_HOST_CLOCK	150000000 /* 150 MHz */

#define C45_MDIO_MCR_OP_ADDR		0x0
#define C45_MDIO_MCR_OP_DATA		0x1
#define C45_MDIO_MCR_OP_DATA_INC_WO	0x3
#define C45_MDIO_MCR_OP_DATA_INC_RW	0x2

#define C22_REG13_NUM		13
#define C22_REG14_NUM		14

/* Write to C22 register 13 the requested Op Code and Device Address (DEVAD).
 * The DEVAD is the upper 16 bits of the regnum passed in from the core. */
#define C22_REG13_MASK(op, reg) \
	(((op & 0x3) << (XAE_MDIO_MCR_OP_SHIFT + 1)) \
	| ((reg >> MII_DEVADDR_C45_SHIFT) & 0x1F))

/* Prototypes of read and write functions for other functions in this file to
 * use. */
static int axienet_mdio_write(struct mii_bus *bus, int phy_id, int reg,
			      u16 val);
static int axienet_mdio_read(struct mii_bus *bus, int phy_id, int reg);

/* Wait till MDIO interface is ready to accept a new transaction.*/
int axienet_mdio_wait_until_ready(struct axienet_local *lp)
{
	u32 val;

	return readx_poll_timeout(axinet_ior_read_mcr, lp,
				  val, val & XAE_MDIO_MCR_READY_MASK,
				  1, 20000);
}

/**
 * axienet_mdio_c45_helper - Help write C22 Registers 13 and 14 the
 * Address and Data Op Codes needed for reading and writing the C45
 * registers.
 * @bus:	Pointer to mii bus structure
 * @phy_id:	Address of the PHY device
 * 
 * Return:	0 on success, -ETIMEDOUT on a timeout
 */
static int axienet_mdio_c45_helper(struct mii_bus *bus, int phy_id, int reg)
{
	int ret;
	struct axienet_local *lp = bus->priv;

	/* Write to C22 Register 13 the C45 Address Op Code and the device
	 * address (DEVAD). */
	ret = axienet_mdio_write(bus, 
			phy_id,
			C22_REG13_NUM,
			C22_REG13_MASK(C45_MDIO_MCR_OP_ADDR, reg));
	if (ret < 0)
		return ret;

	/* Write the C45 register address to C22 Register 14. */
	ret = axienet_mdio_write(bus, phy_id, C22_REG14_NUM, (reg & MII_REGADDR_C45_MASK));
	if (ret < 0)
		return ret;

	/* Write to C22 Register 13 the Data OP Code and DEVAD. */
	ret = axienet_mdio_write(bus, 
			phy_id,
			C22_REG13_NUM,
			C22_REG13_MASK(C45_MDIO_MCR_OP_DATA, reg));
	if (ret < 0)
		return ret;

	ret = axienet_mdio_wait_until_ready(lp);
	return ret;
}

/**
 * axienet_mdio_read - MDIO interface read function
 * @bus:	Pointer to mii bus structure
 * @phy_id:	Address of the PHY device
 * @reg:	PHY register to read
 *
 * Return:	The register contents on success, -ETIMEDOUT on a timeout
 *
 * Reads the contents of the requested register from the requested PHY
 * address by first writing the details into MCR register. After a while
 * the register MRD is read to obtain the PHY register content.
 */
static int axienet_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	u32 rc;
	int ret;
	struct axienet_local *lp = bus->priv;

	if(reg & MII_ADDR_C45)
	{
		ret = axienet_mdio_c45_helper(bus, phy_id, reg);
		if (ret < 0)
			return ret;

		/* Read from C22 Register 14 the C45 register contents. */
		axienet_iow(lp, XAE_MDIO_MCR_OFFSET,
				(((phy_id << XAE_MDIO_MCR_PHYAD_SHIFT) &
				XAE_MDIO_MCR_PHYAD_MASK) |
				((C22_REG14_NUM << XAE_MDIO_MCR_REGAD_SHIFT) &
				XAE_MDIO_MCR_REGAD_MASK) |
				XAE_MDIO_MCR_INITIATE_MASK |
				XAE_MDIO_MCR_OP_READ_MASK));
	}
	else
	{
		ret = axienet_mdio_wait_until_ready(lp);
		if (ret < 0)
			return ret;

		axienet_iow(lp, XAE_MDIO_MCR_OFFSET,
				(((phy_id << XAE_MDIO_MCR_PHYAD_SHIFT) &
				XAE_MDIO_MCR_PHYAD_MASK) |
				((reg << XAE_MDIO_MCR_REGAD_SHIFT) &
				XAE_MDIO_MCR_REGAD_MASK) |
				XAE_MDIO_MCR_INITIATE_MASK |
				XAE_MDIO_MCR_OP_READ_MASK));
	}


	ret = axienet_mdio_wait_until_ready(lp);
	if (ret < 0)
		return ret;

	rc = axienet_ior(lp, XAE_MDIO_MRD_OFFSET) & 0x0000FFFF;

	dev_dbg(lp->dev, "axienet_mdio_read(phy_id=%i, reg=%x) == %x\n",
		phy_id, reg, rc);

	return rc;
}

/**
 * axienet_mdio_write - MDIO interface write function
 * @bus:	Pointer to mii bus structure
 * @phy_id:	Address of the PHY device
 * @reg:	PHY register to write to
 * @val:	Value to be written into the register
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout
 *
 * Writes the value to the requested register by first writing the value
 * into MWD register. The the MCR register is then appropriately setup
 * to finish the write operation.
 */
static int axienet_mdio_write(struct mii_bus *bus, int phy_id, int reg,
			      u16 val)
{
	int ret;
	struct axienet_local *lp = bus->priv;

	dev_dbg(lp->dev, "axienet_mdio_write(phy_id=%i, reg=%x, val=%x)\n",
		phy_id, reg, val);

	ret = axienet_mdio_wait_until_ready(lp);
	if (ret < 0)
		return ret;

	if(reg & MII_ADDR_C45)
	{
		/* Write to C22 Registers 13 and 14 the Address and Data Op codes. */
		ret = axienet_mdio_c45_helper(bus, phy_id, reg);
		if (ret < 0)
			return ret;

		/* Write to C22 Register 14 the C45 register value. */
		axienet_iow(lp, XAE_MDIO_MWD_OFFSET, (u32) val);
		axienet_iow(lp, XAE_MDIO_MCR_OFFSET,
				(((phy_id << XAE_MDIO_MCR_PHYAD_SHIFT) &
				XAE_MDIO_MCR_PHYAD_MASK) |
				((C22_REG14_NUM << XAE_MDIO_MCR_REGAD_SHIFT) &
				XAE_MDIO_MCR_REGAD_MASK) |
				XAE_MDIO_MCR_INITIATE_MASK |
				XAE_MDIO_MCR_OP_WRITE_MASK));

	}
	else
	{
		axienet_iow(lp, XAE_MDIO_MWD_OFFSET, (u32) val);
		axienet_iow(lp, XAE_MDIO_MCR_OFFSET,
				(((phy_id << XAE_MDIO_MCR_PHYAD_SHIFT) &
				XAE_MDIO_MCR_PHYAD_MASK) |
				((reg << XAE_MDIO_MCR_REGAD_SHIFT) &
				XAE_MDIO_MCR_REGAD_MASK) |
				XAE_MDIO_MCR_INITIATE_MASK |
				XAE_MDIO_MCR_OP_WRITE_MASK));
	}


	ret = axienet_mdio_wait_until_ready(lp);
	if (ret < 0)
		return ret;
	return 0;
}

/**
 * axienet_mdio_enable - MDIO hardware setup function
 * @lp:		Pointer to axienet local data structure.
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout.
 *
 * Sets up the MDIO interface by initializing the MDIO clock and enabling the
 * MDIO interface in hardware.
 **/
int axienet_mdio_enable(struct axienet_local *lp)
{
	u32 clk_div, host_clock;

	if (lp->clk) {
		host_clock = clk_get_rate(lp->clk);
	} else {
		struct device_node *np1;

		/* Legacy fallback: detect CPU clock frequency and use as AXI
		 * bus clock frequency. This only works on certain platforms.
		 */
		np1 = of_find_node_by_name(NULL, "cpu");
		if (!np1) {
			netdev_warn(lp->ndev, "Could not find CPU device node.\n");
			host_clock = DEFAULT_HOST_CLOCK;
		} else {
			int ret = of_property_read_u32(np1, "clock-frequency",
						       &host_clock);
			if (ret) {
				netdev_warn(lp->ndev, "CPU clock-frequency property not found.\n");
				host_clock = DEFAULT_HOST_CLOCK;
			}
			of_node_put(np1);
		}
		netdev_info(lp->ndev, "Setting assumed host clock to %u\n",
			    host_clock);
	}

	/* clk_div can be calculated by deriving it from the equation:
	 * fMDIO = fHOST / ((1 + clk_div) * 2)
	 *
	 * Where fMDIO <= 2500000, so we get:
	 * fHOST / ((1 + clk_div) * 2) <= 2500000
	 *
	 * Then we get:
	 * 1 / ((1 + clk_div) * 2) <= (2500000 / fHOST)
	 *
	 * Then we get:
	 * 1 / (1 + clk_div) <= ((2500000 * 2) / fHOST)
	 *
	 * Then we get:
	 * 1 / (1 + clk_div) <= (5000000 / fHOST)
	 *
	 * So:
	 * (1 + clk_div) >= (fHOST / 5000000)
	 *
	 * And finally:
	 * clk_div >= (fHOST / 5000000) - 1
	 *
	 * fHOST can be read from the flattened device tree as property
	 * "clock-frequency" from the CPU
	 */

	clk_div = (host_clock / (MAX_MDIO_FREQ * 2)) - 1;
	/* If there is any remainder from the division of
	 * fHOST / (MAX_MDIO_FREQ * 2), then we need to add
	 * 1 to the clock divisor or we will surely be above 2.5 MHz
	 */
	if (host_clock % (MAX_MDIO_FREQ * 2))
		clk_div++;

	netdev_dbg(lp->ndev,
		   "Setting MDIO clock divisor to %u/%u Hz host clock.\n",
		   clk_div, host_clock);

	axienet_iow(lp, XAE_MDIO_MC_OFFSET, clk_div | XAE_MDIO_MC_MDIOEN_MASK);

	return axienet_mdio_wait_until_ready(lp);
}

/**
 * axienet_mdio_disable - MDIO hardware disable function
 * @lp:		Pointer to axienet local data structure.
 *
 * Disable the MDIO interface in hardware.
 **/
void axienet_mdio_disable(struct axienet_local *lp)
{
	axienet_iow(lp, XAE_MDIO_MC_OFFSET, 0);
}

/**
 * axienet_mdio_setup - MDIO setup function
 * @lp:		Pointer to axienet local data structure.
 *
 * Return:	0 on success, -ETIMEDOUT on a timeout, -ENOMEM when
 *		mdiobus_alloc (to allocate memory for mii bus structure) fails.
 *
 * Sets up the MDIO interface by initializing the MDIO clock and enabling the
 * MDIO interface in hardware. Register the MDIO interface.
 **/
int axienet_mdio_setup(struct axienet_local *lp)
{
	struct device_node *mdio_node;
	struct mii_bus *bus;
	int ret;

	ret = axienet_mdio_enable(lp);
	if (ret < 0)
		return ret;

	bus = mdiobus_alloc();
	if (!bus)
		return -ENOMEM;

	snprintf(bus->id, MII_BUS_ID_SIZE, "axienet-%.8llx",
		 (unsigned long long)lp->regs_start);

	bus->priv = lp;
	bus->name = "Xilinx Axi Ethernet MDIO";
	bus->read = axienet_mdio_read;
	bus->write = axienet_mdio_write;
	bus->parent = lp->dev;
	lp->mii_bus = bus;

	mdio_node = of_get_child_by_name(lp->dev->of_node, "mdio");
	ret = of_mdiobus_register(bus, mdio_node);
	of_node_put(mdio_node);
	if (ret) {
		mdiobus_free(bus);
		lp->mii_bus = NULL;
		return ret;
	}
	return 0;
}

/**
 * axienet_mdio_teardown - MDIO remove function
 * @lp:		Pointer to axienet local data structure.
 *
 * Unregisters the MDIO and frees any associate memory for mii bus.
 */
void axienet_mdio_teardown(struct axienet_local *lp)
{
	mdiobus_unregister(lp->mii_bus);
	mdiobus_free(lp->mii_bus);
	lp->mii_bus = NULL;
}
