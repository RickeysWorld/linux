/*
 * sdhci-dove.c Support for SDHCI on Marvell's Dove SoC
 *
 * Author: Saeed Bishara <saeed@marvell.com>
 *	   Mike Rapoport <mike@compulab.co.il>
 * Based on sdhci-cns3xxx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <mach/sdhci.h>

#include "sdhci-pltfm.h"

irqreturn_t sdhci_dove_gpio_irq(int irq, void *dev_id)
{
	struct sdhci_host* host = dev_id;
#ifdef VERBOSE
	DBG("*** %s got gpio interrupt\n",
		mmc_hostname(host->mmc));
#endif

#ifdef VERBOSE
	sdhci_dumpregs(host);
#endif
	mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}

static void enable_sdio_gpio_irq(struct sdhci_host *host, int enable)
{
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	struct sdhci_dove_platform_data *plat = mv_host->priv;
	unsigned long flags;
	struct sdhci_dove_int_wa *wa_info;

	if (!plat->sdhci_wa)
		return;

	wa_info = plat->sdhci_wa;
	spin_lock_irqsave(&host->lock, flags);

	if (enable) {
		if (wa_info->status == 0) {
			enable_irq(wa_info->irq);
			wa_info->status = 1;
		}
	} else {
		if (wa_info->status == 1) {
			disable_irq_nosync(wa_info->irq);
			wa_info->status = 0;
		}
	}

	mmiowb();

	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_sdio_gpio_irq_enable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	struct sdhci_dove_platform_data *plat = mv_host->priv;
	u32 mpp_ctrl4;

	if (!plat || !plat->sdhci_wa)
		return;

	mpp_ctrl4 = readl(DOVE_MPP_CTRL4_VIRT_BASE);
	mpp_ctrl4 |= 1 << plat->sdhci_wa->func_select_bit;
	writel(mpp_ctrl4, DOVE_MPP_CTRL4_VIRT_BASE);

	mmiowb();
}

static void sdhci_sdio_gpio_irq_disable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *mv_host = sdhci_priv(host);
	struct sdhci_dove_platform_data *plat = mv_host->priv;
	u32 mpp_ctrl4;

	if (!plat || !plat->sdhci_wa)
		return;

	mpp_ctrl4 = readl(DOVE_MPP_CTRL4_VIRT_BASE);
	mpp_ctrl4 &= ~(1 << plat->sdhci_wa->func_select_bit);
	writel(mpp_ctrl4, DOVE_MPP_CTRL4_VIRT_BASE);

	mmiowb();
}

static irqreturn_t sdhci_dove_carddetect_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;

	tasklet_schedule(&host->card_tasklet);
	return IRQ_HANDLED;
};

static u16 sdhci_dove_readw(struct sdhci_host *host, int reg)
{
	u16 ret;

	switch (reg) {
	case SDHCI_HOST_VERSION:
	case SDHCI_SLOT_INT_STATUS:
		/* those registers don't exist */
		return 0;
	default:
		ret = readw(host->ioaddr + reg);
	}
	return ret;
}

static u32 sdhci_dove_readl(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_dove_platform_data *plat = pltfm_host->priv;
	u32 ret;

	switch (reg) {
	case SDHCI_CAPABILITIES:
		ret = readl(host->ioaddr + reg);
		/* Mask the support for 3.0V */
		ret &= ~SDHCI_CAN_VDD_300;
		break;
	case SDHCI_PRESENT_STATE:
		ret = readl(host->ioaddr + reg);
		if (plat && gpio_is_valid(plat->gpio_cd)) {
			if (gpio_get_value(plat->gpio_cd) == 0)
				ret |= SDHCI_CARD_PRESENT;
			else
				ret &= ~SDHCI_CARD_PRESENT;
		}
		break;
default:
		ret = readl(host->ioaddr + reg);
	}
	return ret;
}

static struct sdhci_ops sdhci_dove_ops = {
	.gpio_irq_enable = sdhci_sdio_gpio_irq_enable,
	.gpio_irq_disable = sdhci_sdio_gpio_irq_disable,
	.read_w	= sdhci_dove_readw,
	.read_l	= sdhci_dove_readl,
};

static struct sdhci_pltfm_data sdhci_dove_pdata = {
	.ops	= &sdhci_dove_ops,
	.quirks	= SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER |
		  SDHCI_QUIRK_NO_BUSY_IRQ |
		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_FORCE_DMA |
		  SDHCI_QUIRK_NO_HISPD_BIT,
};

static int __devinit sdhci_dove_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_dove_platform_data *plat;
	int ret;

	ret = sdhci_pltfm_register(pdev, &sdhci_dove_pdata);
	if (ret)
		return ret;

	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);
	plat = pdev->dev.platform_data;
	pltfm_host->priv = plat;

	if (plat) {
		if (plat->sdhci_wa) {
			sdhci_dove_pdata.ops->enable_sdio_irq = enable_sdio_gpio_irq;
			ret = devm_request_irq(&pdev->dev, plat->sdhci_wa->irq,
					sdhci_dove_gpio_irq, IRQF_TRIGGER_LOW,
					mmc_hostname(host->mmc), host);
			if (ret) {
				dev_err(&pdev->dev, "cannot request wa irq\n");
				goto dove_probe_err_irq;
			}
			disable_irq_nosync(plat->sdhci_wa->irq);
		}

		/* Not all platforms can gate the clock, so it is not
		   an error if the clock does not exists. */
		plat->clk = clk_get(&pdev->dev, NULL);
		if (!IS_ERR(plat->clk))
			clk_prepare_enable(plat->clk);

		if (!gpio_is_valid(plat->gpio_cd))
			goto dove_probe_done;

		ret = gpio_request(plat->gpio_cd, "sdhci-cd");
		if (ret) {
			dev_err(mmc_dev(host->mmc), "carddetect gpio request failed\n");
			goto dove_probe_err_cd_req;
		}
		gpio_direction_input(plat->gpio_cd);

		ret = request_irq(gpio_to_irq(plat->gpio_cd), sdhci_dove_carddetect_irq,
					      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					      mmc_hostname(host->mmc), host);
		if (ret) {
			dev_err(mmc_dev(host->mmc), "carddetect irq request failed\n");
			goto dove_probe_err_cd_irq;
		}
	}

dove_probe_done:
	return 0;

dove_probe_err_cd_irq:
	if (plat && gpio_is_valid(plat->gpio_cd))
		gpio_free(plat->gpio_cd);
dove_probe_err_cd_req:
	devm_free_irq(&pdev->dev, plat->sdhci_wa->irq, host);
dove_probe_err_irq:
	sdhci_pltfm_unregister(pdev);
	return ret;
}

static int __devexit sdhci_dove_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_dove_platform_data *plat = pdev->dev.platform_data;

	if (plat) {
		if (gpio_is_valid(plat->gpio_cd)) {
			free_irq(gpio_to_irq(plat->gpio_cd), host);
			gpio_free(plat->gpio_cd);
		}
		if (!IS_ERR(plat->clk))
			clk_disable_unprepare(plat->clk);
	}
	return 0;
}

static struct platform_driver sdhci_dove_driver = {
	.driver		= {
		.name	= "sdhci-dove",
		.owner	= THIS_MODULE,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_dove_probe,
	.remove		= __devexit_p(sdhci_dove_remove),
};

module_platform_driver(sdhci_dove_driver);

MODULE_DESCRIPTION("SDHCI driver for Dove");
MODULE_AUTHOR("Saeed Bishara <saeed@marvell.com>, "
	      "Mike Rapoport <mike@compulab.co.il>");
MODULE_LICENSE("GPL v2");
