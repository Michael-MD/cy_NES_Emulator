import argparse
import sys
import os
import tkinter
from tkinter import filedialog

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from cy_NES_Emulator.NES import *

ftypes = [
    ('All files', '.nes'),
]

def search_for_file_path ():
    currdir = os.getcwd()
    return filedialog.askopenfilename(parent=root, initialdir=currdir, title='Select ROM', filetypes=ftypes)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="cy_NES_Emulator Command Line Utility")
    parser.add_argument("-run", type=str, help="Path to ROM")

    args = parser.parse_args()

    if args.run:
        rom_directory = args.run
    else:
        root = tkinter.Tk()
        root.withdraw() #use to hide tkinter window
        rom_directory = search_for_file_path()

            
    nes = NES(rom_directory)
    nes.run()
    
