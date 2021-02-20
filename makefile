#************************************************************************************************
# \author : Marco Florenzi
# \Date : 20 Feb 2021
#************************************************************************************************

# compiler
COMP = g++ 

# module 
NAME = VRPSolve

# basic directory  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
DIR = ./

# compiler options & sys
SYS = -std=gnu++11 -O0 -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
OPTS = -lconcert -lilocplex -lcplex -lm -lpthread -ldl


# external libreries that Cplex needs
# Linux
CPLX = /opt/ibm/ILOG/CPLEX_Studio201/cplex/
CONC = /opt/ibm/ILOG/CPLEX_Studio201/concert/


# CPLEX inludes and libreries  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
CPLXINC = -I$(CPLX)include

CONCINC = -I$(CONC)include

CPLXLIB = -L$(CPLX)lib/x86-64_linux/static_pic 

CONCLIB = -L$(CONC)lib/x86-64_linux/static_pic


# linking & compiling - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
$(DIR)$(NAME):
	$(COMP) $(SYS) main.cpp $(CPLXINC) $(CONCINC) $(CPLXLIB) $(CONCLIB) -o $(NAME) $(OPTS)



#************************************************************************************************
#****************************************end-file************************************************
#************************************************************************************************
