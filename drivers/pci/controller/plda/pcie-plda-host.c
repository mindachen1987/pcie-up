// SPDX-License-Identifier: GPL-2.0
/*
 * PLDA PCIe XpressRich host controller driver
 *
 * Copyright (C) 2023 Microchip Co. Ltd
 *		      StarFive Co. Ltd.
 *
 * Author: Daire McNamara <daire.mcnamara@microchip.com>
 * Author: Minda Chen <minda.chen@starfivetech.com>
 */

#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci_regs.h>
#include <linux/pci-ecam.h>
#include <linux/platform_device.h>

#include "pcie-plda.h"

static char *err_event_string[PLDA_NUM_NODMA_EVENT] = {
	"axi write request error",
	"axi read request error",
	"axi read timeout",
	NULL, /* door bell event */
	"pcie write request error",
	"pcie read request error",
	"pcie read timeout",
	NULL, /* pcie door event */
	NULL, NULL, NULL, NULL, /* int a-d */
	NULL, /*msi int */
	"aer event",
	"pm/ltr/hotplug event",
	"system error"
};

void __iomem *plda_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
				int where)
{
	struct plda_pcie *pcie = bus->sysdata;

	return pcie->config_base + PCIE_ECAM_OFFSET(bus->number, devfn, where);
}
EXPORT_SYMBOL_GPL(plda_pcie_map_bus);

void plda_pcie_enable_msi(struct plda_pcie *port)
{
	struct plda_msi *msi = &port->msi;
	void __iomem *base = port->bridge_addr;
	u16 reg;
	u8 queue_size;

	/* Fixup MSI enable flag */
	reg = readw_relaxed(base + MSI_CAP_CTRL_OFFSET + PCI_MSI_FLAGS);
	reg |= PCI_MSI_FLAGS_ENABLE;
	writew_relaxed(reg, base + MSI_CAP_CTRL_OFFSET + PCI_MSI_FLAGS);

	/* Fixup PCI MSI queue flags */
	queue_size = reg & PCI_MSI_FLAGS_QMASK;
	queue_size >>= 1;
	reg &= ~PCI_MSI_FLAGS_QSIZE;
	reg |= queue_size << 4;
	writew_relaxed(reg, base + MSI_CAP_CTRL_OFFSET + PCI_MSI_FLAGS);

	writel_relaxed(lower_32_bits(msi->vector_phy),
		       base + MSI_CAP_CTRL_OFFSET + PCI_MSI_ADDRESS_LO);
	writel_relaxed(upper_32_bits(msi->vector_phy),
		       base + MSI_CAP_CTRL_OFFSET + PCI_MSI_ADDRESS_HI);
}
EXPORT_SYMBOL_GPL(plda_pcie_enable_msi);

void plda_handle_msi(struct plda_pcie *port)
{
	struct device *dev = port->dev;
	struct plda_msi *msi = &port->msi;
	void __iomem *bridge_base_addr = port->bridge_addr;
	unsigned long status;
	u32 bit;
	int ret;

	status = readl_relaxed(bridge_base_addr + ISTATUS_LOCAL);
	if (status & PM_MSI_INT_MSI_MASK) {
		writel_relaxed(BIT(PM_MSI_INT_MSI_SHIFT), bridge_base_addr + ISTATUS_LOCAL);
		status = readl_relaxed(bridge_base_addr + ISTATUS_MSI);
		for_each_set_bit(bit, &status, msi->num_vectors) {
			ret = generic_handle_domain_irq(msi->dev_domain, bit);
			if (ret)
				dev_err_ratelimited(dev, "bad MSI IRQ %d\n",
						    bit);
		}
	}
}
EXPORT_SYMBOL_GPL(plda_handle_msi);

static void plda_msi_bottom_irq_ack(struct irq_data *data)
{
	struct plda_pcie *port = irq_data_get_irq_chip_data(data);
	void __iomem *bridge_base_addr = port->bridge_addr;
	u32 bitpos = data->hwirq;

	writel_relaxed(BIT(bitpos), bridge_base_addr + ISTATUS_MSI);
}

static void plda_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct plda_pcie *port = irq_data_get_irq_chip_data(data);
	phys_addr_t addr = port->msi.vector_phy;

	msg->address_lo = lower_32_bits(addr);
	msg->address_hi = upper_32_bits(addr);
	msg->data = data->hwirq;

	dev_dbg(port->dev, "msi#%x address_hi %#x address_lo %#x\n",
		(int)data->hwirq, msg->address_hi, msg->address_lo);
}

static int plda_msi_set_affinity(struct irq_data *irq_data,
				 const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static struct irq_chip plda_msi_bottom_irq_chip = {
	.name = "PLDA MSI",
	.irq_ack = plda_msi_bottom_irq_ack,
	.irq_compose_msi_msg = plda_compose_msi_msg,
	.irq_set_affinity = plda_msi_set_affinity,
};

static int plda_irq_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				     unsigned int nr_irqs, void *args)
{
	struct plda_pcie *port = domain->host_data;
	struct plda_msi *msi = &port->msi;
	void __iomem *bridge_base_addr = port->bridge_addr;
	unsigned long bit;
	u32 val;

	mutex_lock(&msi->lock);
	bit = find_first_zero_bit(msi->used, msi->num_vectors);
	if (bit >= msi->num_vectors) {
		mutex_unlock(&msi->lock);
		return -ENOSPC;
	}

	set_bit(bit, msi->used);

	irq_domain_set_info(domain, virq, bit, &plda_msi_bottom_irq_chip,
			    domain->host_data, handle_edge_irq, NULL, NULL);

	/* Enable MSI interrupts */
	val = readl_relaxed(bridge_base_addr + IMASK_LOCAL);
	val |= PM_MSI_INT_MSI_MASK;
	writel_relaxed(val, bridge_base_addr + IMASK_LOCAL);

	mutex_unlock(&msi->lock);

	return 0;
}

static void plda_irq_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				     unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct plda_pcie *port = irq_data_get_irq_chip_data(d);
	struct plda_msi *msi = &port->msi;

	mutex_lock(&msi->lock);

	if (test_bit(d->hwirq, msi->used))
		__clear_bit(d->hwirq, msi->used);
	else
		dev_err(port->dev, "trying to free unused MSI%lu\n", d->hwirq);

	mutex_unlock(&msi->lock);
}

static const struct irq_domain_ops msi_domain_ops = {
	.alloc	= plda_irq_msi_domain_alloc,
	.free	= plda_irq_msi_domain_free,
};

static struct irq_chip plda_msi_irq_chip = {
	.name = "PLDA PCIe MSI",
	.irq_ack = irq_chip_ack_parent,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static struct msi_domain_info plda_msi_domain_info = {
	.flags = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_PCI_MSIX),
	.chip = &plda_msi_irq_chip,
};

int plda_allocate_msi_domains(struct plda_pcie *port)
{
	struct device *dev = port->dev;
	struct fwnode_handle *fwnode = of_node_to_fwnode(dev->of_node);
	struct plda_msi *msi = &port->msi;

	raw_spin_lock_init(&port->lock);
	mutex_init(&port->msi.lock);

	msi->dev_domain = irq_domain_add_linear(NULL, msi->num_vectors,
						&msi_domain_ops, port);
	if (!msi->dev_domain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	msi->msi_domain = pci_msi_create_irq_domain(fwnode, &plda_msi_domain_info,
						    msi->dev_domain);
	if (!msi->msi_domain) {
		dev_err(dev, "failed to create MSI domain\n");
		irq_domain_remove(msi->dev_domain);
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(plda_allocate_msi_domains);

void plda_handle_intx(struct plda_pcie *port)
{
	struct device *dev = port->dev;
	unsigned long status;
	u32 bit;
	int ret;

	status = readl_relaxed(port->bridge_addr + ISTATUS_LOCAL);
	if (status & PM_MSI_INT_INTX_MASK) {
		status &= PM_MSI_INT_INTX_MASK;
		status >>= PM_MSI_INT_INTX_SHIFT;
		for_each_set_bit(bit, &status, PCI_NUM_INTX) {
			ret = generic_handle_domain_irq(port->intx_domain, bit);
			if (ret)
				dev_err_ratelimited(dev, "bad INTx IRQ %d %d\n",
						    bit, ret);
			if (port->clr_intx_irq)
				writel_relaxed(BIT(bit + PM_MSI_INT_INTX_SHIFT),
					       port->bridge_addr + ISTATUS_LOCAL);
		}
	}
}
EXPORT_SYMBOL_GPL(plda_handle_intx);

static void plda_ack_intx_irq(struct irq_data *data)
{
	struct plda_pcie *port = irq_data_get_irq_chip_data(data);
	void __iomem *bridge_base_addr = port->bridge_addr;
	u32 mask = BIT(data->hwirq + PM_MSI_INT_INTX_SHIFT);

	writel_relaxed(mask, bridge_base_addr + ISTATUS_LOCAL);
}

static void plda_mask_intx_irq(struct irq_data *data)
{
	struct plda_pcie *port = irq_data_get_irq_chip_data(data);
	void __iomem *bridge_base_addr = port->bridge_addr;
	unsigned long flags;
	u32 mask = BIT(data->hwirq + PM_MSI_INT_INTX_SHIFT);
	u32 val;

	raw_spin_lock_irqsave(&port->lock, flags);
	val = readl_relaxed(bridge_base_addr + IMASK_LOCAL);
	val &= ~mask;
	writel_relaxed(val, bridge_base_addr + IMASK_LOCAL);
	raw_spin_unlock_irqrestore(&port->lock, flags);
}

static void plda_unmask_intx_irq(struct irq_data *data)
{
	struct plda_pcie *port = irq_data_get_irq_chip_data(data);
	void __iomem *bridge_base_addr = port->bridge_addr;
	unsigned long flags;
	u32 mask = BIT(data->hwirq + PM_MSI_INT_INTX_SHIFT);
	u32 val;

	raw_spin_lock_irqsave(&port->lock, flags);
	val = readl_relaxed(bridge_base_addr + IMASK_LOCAL);
	val |= mask;
	writel_relaxed(val, bridge_base_addr + IMASK_LOCAL);
	raw_spin_unlock_irqrestore(&port->lock, flags);
}

static struct irq_chip plda_intx_irq_chip = {
	.name = "PLDA PCIe INTx",
	.irq_ack = plda_ack_intx_irq,
	.irq_mask = plda_mask_intx_irq,
	.irq_unmask = plda_unmask_intx_irq,
};

static int plda_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &plda_intx_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = plda_pcie_intx_map,
};

static void plda_handle_event(struct plda_pcie *port)
{
	struct device *dev = port->dev;
	unsigned long status, bits;
	u32 bit;

	status = readl_relaxed(port->bridge_addr + ISTATUS_LOCAL);
	if (status & INT_EVENT_MASK) {
		if (status & DMA_ERR_MASK)
			dev_err_ratelimited(dev, "DMA chan error %lx\n", status);

		bits = status >> A_ATR_EVT_POST_ERR_SHIFT;
		for_each_set_bit(bit, &bits, PLDA_NUM_NODMA_EVENT) {
			if (err_event_string[bit])
				dev_err_ratelimited(dev, "%s\n", err_event_string[bit]);
		}
		writel_relaxed(status & INT_EVENT_MASK,
			       port->bridge_addr + ISTATUS_LOCAL);
	}
}

static void plda_pcie_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct plda_pcie *plda;

	plda = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);

	plda_handle_intx(plda);

	plda_handle_msi(plda);

	if (plda->ops->handle_events)
		plda->ops->handle_events(plda);
	else
		plda_handle_event(plda);

	chained_irq_exit(chip, desc);
}

int plda_init_intx_irq_domain(struct plda_pcie *plda, struct device *dev,
			      struct device_node *intc_node)
{
	struct device_node *node;

	if (!intc_node)
		node = dev->of_node;

	plda->intx_domain =
		irq_domain_add_linear(node, PCI_NUM_INTX, &intx_domain_ops, plda);

	if (!plda->intx_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(plda_init_intx_irq_domain);

static int plda_pcie_init_irq_domain(struct plda_pcie *plda, int irq)
{
	struct device *dev = plda->dev;

	int ret;

	plda_set_default_msi(&plda->msi);

	ret = plda_allocate_msi_domains(plda);
	if (ret != 0)
		return -ENOMEM;

	ret = plda_init_intx_irq_domain(plda, dev, NULL);
	if (ret)
		return ret;

	irq_set_chained_handler_and_data(irq, plda_pcie_isr, plda);

	return 0;
}

void plda_pcie_setup_window(void __iomem *bridge_base_addr, u32 index,
				     phys_addr_t axi_addr, phys_addr_t pci_addr,
				     size_t size)
{
	u32 atr_sz = ilog2(size) - 1;
	u32 val;

	if (index == 0)
		val = PCIE_CONFIG_INTERFACE;
	else
		val = PCIE_TX_RX_INTERFACE;

	writel(val, bridge_base_addr + (index * ATR_ENTRY_SIZE) +
	       ATR0_AXI4_SLV0_TRSL_PARAM);

	val = lower_32_bits(axi_addr) | (atr_sz << ATR_SIZE_SHIFT) |
			    ATR_IMPL_ENABLE;
	writel_relaxed(val, bridge_base_addr + (index * ATR_ENTRY_SIZE) +
	       ATR0_AXI4_SLV0_SRCADDR_PARAM);

	val = upper_32_bits(axi_addr);
	writel(val, bridge_base_addr + (index * ATR_ENTRY_SIZE) +
	       ATR0_AXI4_SLV0_SRC_ADDR);

	val = lower_32_bits(pci_addr);
	writel(val, bridge_base_addr + (index * ATR_ENTRY_SIZE) +
	       ATR0_AXI4_SLV0_TRSL_ADDR_LSB);

	val = upper_32_bits(pci_addr);
	writel(val, bridge_base_addr + (index * ATR_ENTRY_SIZE) +
	       ATR0_AXI4_SLV0_TRSL_ADDR_UDW);

	val = readl(bridge_base_addr + ATR0_PCIE_WIN0_SRCADDR_PARAM);
	val |= (ATR0_PCIE_ATR_SIZE << ATR0_PCIE_ATR_SIZE_SHIFT);
	writel(val, bridge_base_addr + ATR0_PCIE_WIN0_SRCADDR_PARAM);
	writel(0, bridge_base_addr + ATR0_PCIE_WIN0_SRC_ADDR);
}
EXPORT_SYMBOL_GPL(plda_pcie_setup_window);

int plda_pcie_setup_iomems(struct plda_pcie *port, struct pci_host_bridge *bridge)
{
	void __iomem *bridge_base_addr = port->bridge_addr;
	struct resource_entry *entry;
	u64 pci_addr;
	u32 index = 1;

	resource_list_for_each_entry(entry, &bridge->windows) {
		if (resource_type(entry->res) == IORESOURCE_MEM) {
			pci_addr = entry->res->start - entry->offset;
			plda_pcie_setup_window(bridge_base_addr, index,
					       entry->res->start, pci_addr,
					       resource_size(entry->res));
			index++;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(plda_pcie_setup_iomems);

int plda_pcie_enable_phy(struct device *dev, struct plda_pcie *pcie)
{
	int ret;

	if (!pcie->phy)
		return 0;

	ret = phy_init(pcie->phy);
	if (ret)
		return dev_err_probe(dev, ret,
			"failed to initialize pcie phy\n");

	ret = phy_set_mode(pcie->phy, PHY_MODE_PCIE);
	if (ret) {
		dev_err(dev, "failed to set pcie mode\n");
		goto err_phy_on;
	}

	ret = phy_power_on(pcie->phy);
	if (ret) {
		dev_err(dev, "failed to power on pcie phy\n");
		goto err_phy_on;
	}

	return 0;

err_phy_on:
	phy_exit(pcie->phy);
	return ret;
}

void plda_pcie_disable_phy(struct plda_pcie *pcie)
{
	phy_power_off(pcie->phy);
	phy_exit(pcie->phy);
}

static void plda_pcie_irq_domain_deinit(struct plda_pcie *pcie)
{
	irq_set_chained_handler_and_data(pcie->irq, NULL, NULL);

	irq_domain_remove(pcie->msi.msi_domain);
	irq_domain_remove(pcie->msi.dev_domain);

	irq_domain_remove(pcie->intx_domain);
	if (pcie->event_domain)
		irq_domain_remove(pcie->event_domain);
}

int plda_pcie_host_init(struct plda_pcie *pcie, struct pci_ops *ops)
{
	struct resource *cfg_res;
	struct device *dev = pcie->dev;
	int ret;
	struct pci_host_bridge *bridge;
	struct platform_device *pdev = to_platform_device(dev);

	pcie->bridge_addr =
		devm_platform_ioremap_resource_byname(pdev, "reg");

	if (IS_ERR(pcie->bridge_addr))
		return dev_err_probe(dev, PTR_ERR(pcie->bridge_addr),
			"Failed to map reg memory\n");

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (!cfg_res)
		return dev_err_probe(dev, -ENODEV,
			"Failed to get config memory\n");

	pcie->config_base = devm_ioremap_resource(dev, cfg_res);
	if (IS_ERR(pcie->config_base))
		return dev_err_probe(dev, PTR_ERR(pcie->config_base),
			"Failed to map config memory\n");

	pcie->config_phyaddr = cfg_res->start;
	pcie->cfg_size = resource_size(cfg_res);

	pcie->phy = devm_phy_optional_get(dev, NULL);
	if (IS_ERR(pcie->phy))
		return dev_err_probe(dev, PTR_ERR(pcie->phy),
			"Failed to get pcie phy\n");

	pcie->irq = platform_get_irq(pdev, 0);
	if (pcie->irq < 0)
		return dev_err_probe(dev, -EINVAL,
			"Failed to get IRQ: %d\n", pcie->irq);

	bridge = devm_pci_alloc_host_bridge(dev, 0);
	if (!bridge)
		return -ENOMEM;

	pcie->bridge = bridge;

	ret = plda_pcie_enable_phy(dev, pcie);
	if (ret)
		return ret;

	if (pcie->ops->host_init) {
		ret = pcie->ops->host_init(pcie);
		if (ret)
			goto err_phy_on;
	}

	ret = plda_pcie_init_irq_domain(pcie, pcie->irq);
	if (ret)
		goto err_host;

	/* Set default bus ops */
	bridge->ops = ops;
	bridge->sysdata = pcie;

	plda_pcie_enable_msi(pcie);

	ret = pci_host_probe(bridge);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to pci host probe: %d\n", ret);
		goto err_probe;
	}

	return ret;

err_probe:
	plda_pcie_irq_domain_deinit(pcie);
err_host:
	if (pcie->ops->host_deinit)
		pcie->ops->host_deinit(pcie);
err_phy_on:
	plda_pcie_disable_phy(pcie);
	return ret;
}
EXPORT_SYMBOL_GPL(plda_pcie_host_init);

void plda_pcie_host_deinit(struct plda_pcie *pcie)
{
	pci_stop_root_bus(pcie->bridge->bus);
	pci_remove_root_bus(pcie->bridge->bus);

	plda_pcie_irq_domain_deinit(pcie);

	if (pcie->ops->host_deinit)
		pcie->ops->host_deinit(pcie);

	plda_pcie_disable_phy(pcie);
}
EXPORT_SYMBOL_GPL(plda_pcie_host_deinit);
