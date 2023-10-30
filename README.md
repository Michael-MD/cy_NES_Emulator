# cy_NES_Emulator
This is an emulator I wrote for emulating the nes system using cython. 
This project was heavily inspired from javidx9 (https://www.youtube.com/@javidx9) so thanks to him.


## Usage
### Command Line
To use cy_NES_Emulator from the command line simply open the command line in the same directory as the cy_NES_Emulator folder and type the following
```
python cy_NES_Emulator/main.py
```
You will be prompted to choose the location of your rom file.
You may also specity a path directly using 
```
python cy_NES_Emulator/main.py -run "Super Mario Bros (E).nes"
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


