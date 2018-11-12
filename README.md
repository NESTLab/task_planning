## Multi-Robot Task Planning with Space-Time Constraints for Coordination Tasks

### Pre-requisite
Install IBM CPLEX Optimization Studio:
http://www-01.ibm.com/support/docview.wss?uid=swg27050618
(Free for students)

### Installation & Usage
Clone/download the repository.

#### MATLAB
After installing the CPLEX Optimization Studio, it needs to be configured in the MATLAB environment. Add these paths using the Set Path tab in the MATLAB Search Paths list:
/<yourpath>/ILOG/CPLEX_Studio128/cplex/matlab
/<yourpath>/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux
/<yourpath>/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux/help
/<yourpath>/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux/help/helpsearch-v2
/<yourpath>/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux/help/topics

For more info on using CPLEX in MATLAB:
https://www.ibm.com/support/knowledgecenter/en/SSSA5P_12.6.2/ilog.odms.cplex.help/CPLEX/MATLAB/topics/gs_install.html

#### CPLEX IDE
Open 

Create a new project and use the .mod and .dat files in the repository as the OPL Model and Data files for the project.
Add these two into a new Run Configuration and 
