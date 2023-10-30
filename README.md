# cy_NES_Emulator
Nes system in cython with joystick support. 
This project was heavily inspired from javidx9 (https://www.youtube.com/@javidx9) so thanks to him.

<img src="smb.gif" width="400"/>


### Recommended Setup
I recommended you set this up in an anaconda enviorment. You can download it from here: https://www.anaconda.com/download. After downlaoding open an anaconda prompt and type:
```
conda create --name cy_NES_EMU_env python=3.9
```
Run through the setup procedure as prompted. 
Once complete enter
```
conda activate cy_NES_EMU_env
```
Navigate to the module directory
```
cd path/to/module/cy_NES_Emulator
```
Inside is a requirements file, install it as follows
```
pip install -r requirements.txt
```
Lastly, run the command
```
python setup.py build_ext --inplace
```

To compile the code.

All done! The module behaves exactly like any other python module.

## Usage
### Command Line
To use cy_NES_Emulator from the command line simply open the command line in the same directory as the cy_NES_Emulator folder and type the following
```
python cy_NES_Emulator
```

You will be prompted to choose the location of your rom file.
You may also specity a path directly using

```
python cy_NES_Emulator -run "Super Mario Bros (E).nes"
```

### Within Python file
Simply import as follows
```
from cy_NES_Emulator import NES
```
followed by these two lines
```
nes = NES("Super Mario Bros (E).nes")
nes.run()
```



