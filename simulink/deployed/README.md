# Generated controller

Run the desktop deployment tool or `python tools/deploy_foc.py --install-project --verify-arm` from the repository root. After a successful build, the tool publishes the generated controller to `current/`. EIDE uses that directory for its `Simulink_FOC` target. Make controller edits in `odrive_foc.slx` and regenerate.
