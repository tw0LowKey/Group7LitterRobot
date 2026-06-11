This the introductory README file for the docs folder.

The following folder is seperated into an introductory hardware setup file (Assembly Instructions.docx) and introductory software setup markdown file (Software Setup.md), for the entire system of SAPLInG including both PickBot and BinBot, followed by folders for each subsystem of SAPLInG.

The folders for each subsystem contain informaton on the packages needing to be installed separately to run each subsystem (Dependencies.md), and information for startup of each subsystem (Startup.md). Information on how individual files function can be inferred from code comments and the provided copy of the final report. 

**Initial software setup is recommended to be completed first, followed by the initial hardware setup** as placing the Jetson Orin Nano onto the PickBot means it cannot be easily accessed. Other than this, subsystems can theoretically be implemented in any order, so long as they do  not depend on another subsystem being implemented first. 
