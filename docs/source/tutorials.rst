Tutorials
=========

How to use smc_storm
--------------------

Retrieve the available options
++++++++++++++++++++++++++++++

.. code-block:: bash

    smc_storm --help

Examples
++++++++

Here some examples are provided, to show how to run smc_storm on the exemplary models provided in the `models folder <https://github.com/convince-project/smc_storm/tree/main/test/models>`_:

.. code-block:: bash

    smc_storm --model leader_sync.3-2.v1.jani --properties-names eventually_elected --batch-size 200
    smc_storm --model nand.v1.jani --properties-names reliable --constants "N=20,K=2" --epsilon 0.01 --confidence 0.95 --n-threads 5 --show-statistics
    smc_storm --model leader_sync.3-2.v1.jani --properties-names time --traces-file ~/exported_traces.csv --show-statistics --max-n-traces 5
    smc_storm --model die.jani --custom-property "P=? [F s=7&d=6]"
    smc_storm --model brp.v1.prism --properties-file brp.v1.props --constants "N=16,MAX=2"
