# Simple Processor

A simple, multicycle processor with 4 instructions. Can be implemented in Basys3 FPGA with button a debouncer and a simple display.

## Instruction Set
Instructions used in the processor are 12 bits. The first three bits represent the instruction and may be called as *op*.

### Load

000-XX-R<sub>2</sub>R<sub>1</sub>R<sub>0</sub>-D<sub>3</sub>D<sub>2</sub>D<sub>1</sub>D<sub>0</sub>
: A load instruction's *op* value is 000. Next two bits are don't cares. R<sub>2:0</sub> bits represent the write address of the register file, and D<sub>3:0</sub> bits represent the data memory address. The value DM[D] is written into RF[R].
### Store
001-XX-R<sub>2</sub>R<sub>1</sub>R<sub>0</sub>-D<sub>3</sub>D<sub>2</sub>D<sub>1</sub>D<sub>0</sub>

: A store instruction's *op* value is 001. Next two bits are don't cares. R<sub>2:0</sub> bits represent the read address of the register file, and D<sub>3:0</sub> bits represent the data memory address. The value RF[R] is written into DM[D].
### Add
101-W<sub>2</sub>W<sub>1</sub>W<sub>0</sub>-B<sub>2</sub>B<sub>1</sub>B<sub>0</sub>-A<sub>2</sub>A<sub>1</sub>A<sub>0</sub>

: An add instruction's *op* value is 101. W<sub>2:0</sub> represents the write address of the register file, A<sub>2:0</sub> and B<sub>2:0</sub>, represents the first and second read address port of the register file, respectively. The value RF[A] + RF[B] is written into RF[W].
### Sub
110-W<sub>2</sub>W<sub>1</sub>W<sub>0</sub>-B<sub>2</sub>B<sub>1</sub>B<sub>0</sub>-A<sub>2</sub>A<sub>1</sub>A<sub>0</sub>

: A store instruction's *op* value is 001. W<sub>2:0</sub> represents the write address of the register file, A<sub>2:0</sub> and B<sub>2:0</sub>, represents the first and second read address port of the register file, respectively. The value RF[A] - RF[B] is written into RF[W].

## Display
When an instruction is executed, important data is displayed in the sevent segment display. What is displayed differed in different instructions.

### Load & Store
The 4-bit data being read/stored is shown in the display. For instance, if the instruction reads 4 from the data memory and writes it back to the register file, the display would show:
````
00-4
````
### Add & Sub
The 4-bit data being added and substracted, and the result of the summation/substraction is shown in the display. For instance, if the instruction reads 5 and 7 from the register file and writes it back to any place in the register file, the display would show:
````
57-C
````
## Push Buttons

- Center
  - Resets the program counter and clears the display. The contents of the memory and register file does not change.
- Left
  - Executes the next instruction in the instruction memory.
- Right
  - Executes the instruction in the switches, located in switches SW<sub>11</sub> to SW<sub>0</sub>.

## Design

### State Machine Diagrams
HLSM Diagram:
![HLSM diagram](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/img/HLSM%20diagram.vpd%20(1).png)

Reduced FSM Diagram:
![Controller FSM](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/img/HLSM%20diagram.vpd%20(1).png)

### Block Diagrams

The processor is connected to 2 button debouncers and the sevent segment display:
![Top Module](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/img/topmost%20module.png)

The processor is composed of two modules, datapath and controller. They are connected to each other like the following:

![Processor](https://dochub.com/zubeyir99/0YkWQ4BwYWzqEj3Kpl7A8q/top-module2-png)



The datapath:

![Datapath](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/img/datapath.png)

The controller:
![Controller](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/img/Controller.png)

Please read more in the [project report](https://github.com/zubeyir-bodur/Simple-Processor/blob/master/Report.pdf) for the contents of the ROM.
