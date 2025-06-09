# Applying the EMG Controller Patch

This directory contains a patch file that shows how to modify `algorithm_ctrl.c` to integrate the EMG controller.

## Manual Integration

To integrate the EMG controller manually:

1. Open `algorithm_ctrl.c` in your editor
2. Add the following include at the top of the file:
   ```c
   #include "robot_emg_controller.h"
   ```

3. Find the USER_DEFINED_CTRL section in the file (around line 377):
   ```c
   else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
       
   }
   ```

4. Add the call to the EMG controller update function:
   ```c
   else if (controlMode == USER_DEFINED_CTRL) {   // 6 - User Defined Control
       // Call the EMG controller update function
       emg_controller_update();
   }
   ```

## Using the Patch File

If you're familiar with patch files, you can apply the patch directly:

```bash
cd Expansion_Board_H743AII6/AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/
patch algorithm_ctrl.c < patch/algorithm_ctrl.c.patch
```

Note: The patch file assumes a specific version of `algorithm_ctrl.c`. If your version is different, you may need to apply the changes manually. 