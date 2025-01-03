# CWT (ComWinTop) IO Card parameter reading/writing tool
## Background
Having used CWT-MB IO cards for data collection, I got annoyed that the only way to change their MODBUS addresses was through an antiquated Windows GUI program.
I've reverse engineered the data payloads sent back and forth between computer and CWT IO Card to be able to read/write/modify MODBUS parameters and analog input settings through Python.

## Repo
CWT_Config folder:
- ConfigEditor.py has functions for reading, modifying, and writing RS232, RS485, and TCP/IP parameters for CWT-MB IO Cards
- AnalogInputEditor.py has functions for reading, modifying, and writing analog input pin settings for CWT-MB IO Cards
- Utils.py has supporting functions used by other files in the folder 

Testing folder:
- Config Protocol.txt and Analog Input Protocol.txt have the CWT Config/AI payloads laid out with definitions of most bytes.  Byte indexes (listed on the left side) without definitions, e.g. just hex values, are unknown for now.  They may mean something or may just be for padding(?).  I've mapped all of the original config program's parameters to bytes, so the unknown bytes are sent in ConfigEditor.py as-is.
- Modbus Testing.txt and AI Config Testing.txt have some sample back-and-forth UART communications between a computer and CWT IO Card using the official config and AI GUI programs (this was used to help reverse-engineer the protocols)

### I am not associated with ComWinTop.  Original config tool is available on their website, [comwintop.com](http://www.comwintop.com/index.php?s=index/category/index&id=142).  All code and findings in this repo are mine