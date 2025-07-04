Installation Guide
==================

How to install smc_storm
------------------------

smc_storm can be obtained in many different ways:

Using deployed binaries
++++++++++++++++++++++++

We provide pre-built binaries that can be used on Ubuntu. They can be found at the `Releases page <https://github.com/convince-project/smc_storm/releases>`_.

To install them on your machine, extract `smc_storm_executable.tar.gz` and follow these steps:

.. code-block:: bash

    cd smc_storm_executable            # This is the extracted archive
    install.sh --install-dependencies  # This flag will install all  packages required by smc_storm and its dependencies
    export PATH=$PATH:$PWD/bin         # This way, smc_storm can be called from anywhere
    smc_storm --help                   # Make sure the binary runs

Build from source
+++++++++++++++++

Install STORM
_____________

smc_storm needs STORM to be built on the local machine. To achieve that, follow the `official documentation <https://www.stormchecker.org/documentation/obtain-storm/build.html>`_.

In order to get the latest features, we recommend using the `master` branch, which provides support for the trigonometric operators that are not supported yet in the `stable` branch.

We used the following command to build STORM:

.. code-block:: bash

    export STORM_DIR=<path-to-storm-repo>
    cmake -DSTORM_USE_SPOT_SHIPPED=ON $STORM_DIR && make -j10

Install smc_verifiable_plugins
______________________________

This is another dependency, providing support for external plugins that can be executed by smc_storm when simulating the model.

The package can be found at `this link <https://github.com/convince-project/smc_verifiable_plugins>_`.

Once cloned, you can use the following:

.. code-block:: bash

    cd <path-to-smc-storm-repo>
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make

Install smc_storm
_________________

Once STORM is installed, smc_storm can be built using the following:

.. code-block:: bash

    cd <path-to-smc-storm-repo>
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4

Get a docker container containing smc_storm
+++++++++++++++++++++++++++++++++++++++++++

You can use the `AS2FM <https://github.com/convince-project/AS2FM>_` docker container, that comes with the latest release of smc_storm already installed.

.. code-block:: bash

    docker pull ghcr.io/convince-project/as2fm:latest
    docker run -it --rm ghcr.io/convince-project/as2fm:latest

Make smc_storm available from anywhere
++++++++++++++++++++++++++++++++++++++

To make it possible to call smc_storm from anywhere in your system, we recommend two possible options:

Option 1: Add a symlink in ~/.local/bin
_______________________________________

.. code-block:: bash

    cd ~/.local/bin
    ln -s ln -s <path-to-smc-storm-binary-folder>/smc_storm .

Option 2: Append it to the PATH env variable
____________________________________________

.. code-block:: bash

    export PATH=$PATH:<path-to-smc-storm-binary-folder>

To avoid doing this each time you open a new terminal, the above line can be added to the `~/.bashrc` file.

The `<path-to-smc-storm-binary-folder>` depends on the chosen installation strategy:
* If you used the pre-built binaries, the path will be `<path-to-workspace>/smc_storm_executable/bin`
* If you built from source, the path will be `<path-to-workspace>/smc_storm/build/bin`

The `<path-to-workspace>` is the directory where you extracted the pre-built binaries or where you cloned the smc_storm repository.

Verify the installation works
-----------------------------

To verify that the binaries are able to execute, you can try to run it and check the execution terminates correctly:

.. code-block:: bash

    smc_storm --help
