##Masters Thesis:\n Designing a new generation system for High Density surface Electromyography  

This firmware is for a new generation HDsEMG system developed for my master's thesis.
The whole project is currently in the dev_datastream branch and will be merged with the main branch in the future. For now the master branch contains the fixed esp32-s3 espnow example from espressif.

The code is divided into multiple .c and .h files. Each .c and .h file is dedicated to one subsystem of the acquisition system.
The designed sytem is divided into two subsystems:
    AFE (Analog Front End) controll
    wireless communication via espnow

The system works currently with a variable length of electrode arrays and works at a sample rate of 1 kSps. Data grouping for wireless trasmission will be implemented in the near future to enable higher sample rates.
This readme should be expanded pretty soon too, but this will do for now.
Come back soon to see how this project continues to grow
