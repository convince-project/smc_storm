Installation Guide
==================

How to install smc_storm
------------------------

Install STORM
+++++++++++++

smc_storm needs STORM to be built on the local machine. To achieve that, follow the `official documentation <https://www.stormchecker.org/documentation/obtain-storm/build.html>`_.

In order to get the latest features, we recommend using the `master` branch, which provides support for the trigonometric operators that are not supported yet in the `stable` branch.

We used the following command to build STORM:

.. code-block:: bash

    export STORM_DIR=<path-to-storm-repo>
    cmake -DSTORM_USE_SPOT_SHIPPED=ON $STORM_DIR && make -j10

Install smc_storm
+++++++++++++++++

Once STORM is installed, smc_storm can be built using the following:

.. code-block:: bash

    cd <path-to-smc-storm-repo>
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4

Make it available from anywhere
+++++++++++++++++++++++++++++++

In order to be able to call smc_storm from anywhere in bash, you can add the smc_storm path to the PATH env variable:

.. code-block:: bash

    export PATH=$PATH:$HOME/storm_ws/smc_storm/build


Verify everything is working
----------------------------

To verify that everything is working, you can run the following test from the build folder:

.. code-block:: bash

    ./test_models
