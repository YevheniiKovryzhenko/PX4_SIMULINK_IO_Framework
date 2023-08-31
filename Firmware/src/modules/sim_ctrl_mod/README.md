# Adding new parameters:
1. open "sim_strl_mod.h"
2. scroll down the file for instructions, add new parameter
3. open "module.yalm", 
	this file is very type-sensitive, so make sure you type everything properly.
	Wrong indentation or extra white space will cause the entire code to fail on-compile-time
4. Add settings to your new parameter using the global name you defined earlier
5. Done. Copy changes to the source firmware and recompile the entire code for changes to take effect.  

For more details on parameters, see this page:
	https://dev.px4.io/v1.11_noredirect/en/advanced/parameters_and_configurations.html
Note, YAML parameters are new so not as well documented, but are quite intuitive to use once you see the other parameters
as examples.

# Starting the module:
Your startup routine should contain the following lines for this app to be started automatically on boot:

	set +e #makes app optional, failure of the app does not cause failure of the firmware
	sim_ctrl_mod start -f -p 42
	set -e