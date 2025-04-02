Reference
=========

Special Features
++++++++++++++++

SMC Plugins
~~~~~~~~~~~

With SMC Storm, we introduce the possibility of having specific actions that are mapped to external plugins, that can be defined using C++, instead of programming them in the JANI format.

Plugins can be developed using the SmcPluginBase class provided in the smc_plugins package.

JANI models making use of plugins, can be executed only by means of its custom cache-less states expansion modality (using the `--disable-explored-states-caching` flag).
Additionally, the paths to the folders containing all required plugins shall be provided when invoking SMC Storm (using `--plugin-paths`).

Here an example for verifying the plugin-based model called `sum_dice_throws_plugins.jani` using smc_storm:

.. code-block:: bash

    smc_storm --model sum_dice_throws_plugins.jani --properties-names sum-to-nine-in-three --show-statistics --plugin-paths \<path-to-smc_storm-lib-folder\>,\<path-to-smc_plugins-lib-folder\> --disable-explored-states-caching


In order to develop a model that makes use of plugins, a specific `plugins` entry needs to be provided in the JANI file.
Its JSON-schema is provided below:

.. code-block:: javascript

    schema({
    "plugins": Array.of(
        {
            "plugin_id": String,                    // Name of plugin to load, related to the plugin implementation
            "automaton_id": String,                 // Name of the automaton associated to the plugin
            "action_name": String,                  // Name of the action in the automaton that advances the plugin to the next step
            "init": Array.of(                       // Configuration (constant) parameters for initializing the plugin
            {
                "name": String,                     // Name of the init config param to configure in the plugin
                "type": ("int", "bool", "real"),
                "value": (int, bool, real)
            }),
            "input": Array.of(                      // Definition of what to provide as input to the plugin
            {
                "name": String,                     // Referring to plugin's internal names
                "type": ("int", "bool", "real"),
                "value": Expression                 // Value passed to the plugin: any valid JANI Expression works
            }),
            "output": Array.of(
            {
                "ref": String,                      // Referring to the JANI variable storing the output value
                "value": String                     // Referring to the plugin's internal name
            }),
        }),
    })

In smc_storm we enforce that the edges associated to the plugins contain only the target locations.
No assignment is allowed in the JANI edges associated to a plugin.
