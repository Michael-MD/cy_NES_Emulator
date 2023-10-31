# cy_NES_Emulator
Nes system in cython with joystick support. 
This project was heavily inspired from javidx9 (https://www.youtube.com/@javidx9) so thanks to him.

<img src="smb.gif" width="400"/>


## Recommended Setup
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

## Controls
The emulator has keyboard and joystick support. If you plug in another controller you can probably guess what the mapping will be.
<table>
	<tr>
		<th> NES controller </th>
		<th> Keyboard </th>
		<th> Xbox 360 controller </th>
		<th> PS3 controller </th>
	</tr>
	<tr>
		<td> A </td>
		<td> x </td>
		<td> A </td>
		<td> X </td>
	</tr>
	<tr>
		<td> B </td>
		<td> z </td>
		<td> B </td>
		<td> ○ </td>
	</tr>
	<tr>
		<td> SELECT </td>
		<td> a </td>
		<td> BACK </td>
		<td> SELECT </td>
	</tr>
	<tr>
		<td> START </td>
		<td> s </td>
		<td> START </td>
		<td> START </td>
	</tr>
	<tr>
		<td> UP </td>
		<td> up </td>
		<td> D-pad UP </td>
		<td> D-pad UP </td>
	</tr>
	<tr>
		<td> DOWN </td>
		<td> down </td>
		<td> D-pad DOWN </td>
		<td> D-pad DOWN </td>
	</tr>
	<tr>
		<td> LEFT </td>
		<td> left </td>
		<td> D-pad LEFT </td>
		<td> D-pad LEFT </td>
	</tr>
	<tr>
		<td> RIGHT </td>
		<td> right </td>
		<td> D-pad RIGHT </td>
		<td> D-pad RIGHT </td>
	</tr>
</table>

