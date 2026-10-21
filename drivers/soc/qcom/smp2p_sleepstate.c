// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm smp2p "sleepstate": tell a remote processor whether the
 * application processor is awake.
 *
 * The sensor DSP (SLPI) firmware reads bit 12 of the "sleepstate" entry
 * that the application processor exports over smp2p. The downstream
 * kernel sets it while the system runs and clears it while it is
 * suspended, so that the DSP can batch non-wakeup sensor data instead of
 * delivering it to a sleeping system.
 */

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/suspend.h>

#define SLEEPSTATE_AWAKE	BIT(12)

struct smp2p_sleepstate {
	struct device *dev;
	struct qcom_smem_state *state;
	struct notifier_block pm_nb;
};

static void smp2p_sleepstate_set(struct smp2p_sleepstate *ss, bool awake)
{
	int ret;

	ret = qcom_smem_state_update_bits(ss->state, SLEEPSTATE_AWAKE,
					  awake ? SLEEPSTATE_AWAKE : 0);
	if (ret)
		dev_err(ss->dev, "failed to update the sleep state: %d\n", ret);
}

static int smp2p_sleepstate_pm_event(struct notifier_block *nb,
				     unsigned long event, void *unused)
{
	struct smp2p_sleepstate *ss = container_of(nb, struct smp2p_sleepstate, pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		smp2p_sleepstate_set(ss, false);
		break;
	case PM_POST_SUSPEND:
		smp2p_sleepstate_set(ss, true);
		break;
	}

	return NOTIFY_DONE;
}

static int smp2p_sleepstate_probe(struct platform_device *pdev)
{
	struct smp2p_sleepstate *ss;
	unsigned int bit;
	int ret;

	ss = devm_kzalloc(&pdev->dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->dev = &pdev->dev;
	ss->state = devm_qcom_smem_state_get(&pdev->dev, NULL, &bit);
	if (IS_ERR(ss->state))
		return dev_err_probe(&pdev->dev, PTR_ERR(ss->state),
				     "failed to get the smem state\n");

	smp2p_sleepstate_set(ss, true);

	ss->pm_nb.notifier_call = smp2p_sleepstate_pm_event;
	ss->pm_nb.priority = INT_MAX;
	ret = register_pm_notifier(&ss->pm_nb);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ss);

	return 0;
}

static void smp2p_sleepstate_remove(struct platform_device *pdev)
{
	struct smp2p_sleepstate *ss = platform_get_drvdata(pdev);

	unregister_pm_notifier(&ss->pm_nb);
}

static const struct of_device_id smp2p_sleepstate_of_match[] = {
	{ .compatible = "qcom,smp2p-sleepstate" },
	{}
};
MODULE_DEVICE_TABLE(of, smp2p_sleepstate_of_match);

static struct platform_driver smp2p_sleepstate_driver = {
	.probe = smp2p_sleepstate_probe,
	.remove = smp2p_sleepstate_remove,
	.driver = {
		.name = "qcom_smp2p_sleepstate",
		.of_match_table = smp2p_sleepstate_of_match,
	},
};
module_platform_driver(smp2p_sleepstate_driver);

MODULE_DESCRIPTION("Qualcomm smp2p sleep state notification");
MODULE_LICENSE("GPL");
