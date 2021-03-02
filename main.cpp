#include <stdio.h>
#include <math.h>
#include <ilcplex/ilocplex.h>

using namespace std;

ILOSTLBEGIN
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix> NumVarMatrixg;
typedef IloArray<NumVarMatrixg> NumVarMatrixg1;
typedef IloArray<NumVarMatrixg> NumVarMatrixg1;
typedef IloArray<IloNumArray> NumMatrix;
typedef IloArray<NumMatrix> NumMatrixg;
typedef IloNumVarArray* NumVar2Matrix;

int main(int argc, char **argv)
{
	
	try{
	
		IloEnv GapEnv; // environment
		char str[128];
		
		IloNum NrClienti; 
  		IloNum NrVeicoli; 
  		IloNum NrDepositi; 
  		IloNumArray Clienti(GapEnv); 
  		IloNumArray Depositi(GapEnv);
  		IloNumArray Veicoli(GapEnv);
  		IloNumArray DomandeClienti(GapEnv); 
		IloNumArray CapienzaVeicoli(GapEnv);
		NumMatrix CostoUnitarioRoutingVeicoli(GapEnv); 
		
		// load data file - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 		//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 		
 		ifstream InFile(argv[1], ios::in);
  		if (!InFile) {
   			cerr << "No such file: " << argv[1] << endl;
   			throw(1);
   		}

		InFile >> NrClienti;
		InFile >> NrVeicoli;
		InFile >> NrDepositi;
		InFile >> Depositi;
		InFile >> Clienti;
		InFile >> DomandeClienti;
		InFile >> CapienzaVeicoli;
		InFile >> CostoUnitarioRoutingVeicoli;
		InFile.close();
			
		
		// construct the GAP model  - - - - - - - - - - - - - - - - - - - - - - -
  		//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  		IloModel GapMod(GapEnv);
  		
  		NumMatrixg μ(GapEnv,NrVeicoli);
  		for(IloInt v = 0; v < NrVeicoli; v++ ){
  			NumMatrix μ1(GapEnv, NrClienti + 1);
  			μ1 = CostoUnitarioRoutingVeicoli;
  			μ[v] = μ1;
  					
  		}	
  	
  		
  		NumVarMatrixg w(GapEnv,NrClienti + 1);
  		
  		
  		for( IloInt i = 0; i < NrClienti + 1 ; i++ ) {
  			NumVarMatrix w1(GapEnv,NrClienti);
  			w[i] = w1;
  			for(IloInt j = 0; j < NrClienti; j++){
  				w[i][j] = IloNumVarArray(GapEnv, NrVeicoli, 0, 1, ILOINT);
  				for( IloInt v = 0; v < NrVeicoli; v++ ) {
    					sprintf(str, "w(%li,%li,%li)",i,j+1,v+1);
    					w[i][j][v].setName(str);
    				}
  			
  			}
  				
   		}
   		
   		NumVarMatrixg1 x(GapEnv,NrClienti + 1);
   		
   		
   		for( IloInt i = 0; i < NrClienti + 1 ; i++ ){
   			NumVarMatrixg x1(GapEnv,NrClienti);
   			x[i] = x1;
   			for(IloInt j = 0; j < NrClienti; j++){
   				NumVarMatrix x2(GapEnv,NrVeicoli);
   				x[i][j] = x2;
   				for(IloInt v = 0; v < NrVeicoli; v++){
   					x[i][j][v] = IloNumVarArray(GapEnv, NrClienti, 0, IloInfinity, ILOINT);
   					for( IloInt k = 0; k < NrClienti; k++ ) {
    						sprintf(str, "x(%li,%li,%li,%li)",i,j+1,v+1,k+1);
    						x[i][j][v][k].setName(str);
    					}		
   				}
   			}
   			
   		}
  			
	   		
		 IloObjective cost_function = IloAdd(GapMod, IloMinimize(GapEnv));
		 IloExpr obj(GapEnv);
		 
		for(IloInt v = 0; v < NrVeicoli; v++){
			for( IloInt i = 0; i < NrClienti + 1; i++ ){
		 	
		 		for(IloInt j = 0; j < NrClienti; j++){
					
					
					
					obj += μ[v][i][j] * w[i][j][v];
						
				}	
			}
			
		}
		cost_function.setExpr(obj);
		
		//Constraints	
		//1
		
                //IloRangeArray vincolo1(GapEnv, NrClienti , DomandeClienti, DomandeClienti));
                IloInt s = 0;
                for(IloInt k = 0; k < NrClienti; k++){
                	IloExpr sumvincolo1(GapEnv);
                	for(IloInt v = 0; v < NrVeicoli; v++){
                		for(IloInt j = 0; j < NrClienti; j++){
                			sumvincolo1 +=x[s][j][v][k];
                		}
                	
                	}   
                	sprintf(str, "constr1(%li)",k);
                	GapMod.add(sumvincolo1 == DomandeClienti[k]).setName(str);          
                }
               
		
		
		//IloRangeArray Knapsack2 = IloAdd(GapMod,
                       // IloRangeArray(GapEnv, DomandeClienti , DomandeClienti ));
                
                
      		//2
      		
               	for(IloInt k=0; k < NrClienti;k++){
               		IloExpr sumvincolo2(GapEnv);
               		for(IloInt v = 0; v < NrVeicoli; v++){
               			for(IloInt i = 0; i < NrClienti + 1; i++){
               				
               				
               				sumvincolo2 += x[i][k][v][k];
        
               			}
               			
               		}
               		
               		sprintf(str, "constr2(%li)",k);
               		GapMod.add(sumvincolo2 == DomandeClienti[k]).setName(str);
               		
               	
               	}
               	
               	
   		//3
   		//forall (kp in Clienti, k in Clienti, v in Veicoli)
 	  	//if(kp != k)
 	    	//sum(<i, kp> in Grafo)(y[<i,kp>][v][k]) - sum(<kp, i> in Grafo)(y[<kp,i>][v][k]) == 0;		
 	    	
 	    	for(IloInt kp =0; kp < NrClienti; kp++){
 	    		for(IloInt k = 0; k < NrClienti; k++){
 	    			for(IloInt v = 0; v < NrVeicoli; v++){
 	    				if(kp != k){
 	    					IloExpr sumvincolo3a(GapEnv);
 	    					IloExpr sumvincolo3b(GapEnv);
 	    					for(IloInt i = 0; i < NrClienti + 1;i++){
 	    						sumvincolo3a += x[i][kp][v][k]; 
 	    					}
 	    					for(IloInt i = 0; i < NrClienti; i++){
 	    						sumvincolo3b += x[kp+1][i][v][k];
 	    					}
 	    					
 	    					sprintf(str, "constr3(%li,%li,%li)",kp,k,v);
 	    					GapMod.add((sumvincolo3a - sumvincolo3b) == 0).setName(str);
 	    				}
 	    			
 	    				
 	    			}
 	    		}
 	    	
 	    	}
 	    	
 	    	//4
 	    	IloInt s1 = 0;
 	    	for(IloInt v = 0; v < NrVeicoli; v++){
 	    		IloExpr sumvincolo4(GapEnv);
 	    		for(IloInt i = 0; i < NrClienti; i++){
 	    			sumvincolo4 += w[s1][i][v];
 	    		
 	    		}
 	    		sprintf(str, "constr4(%li)",v);
 	    		GapMod.add(sumvincolo4 <= 1).setName(str);
 	    	}
 	    	
 	    	//5
 	    	for(IloInt v = 0; v < NrVeicoli; v++){
 	    		for(IloInt k = 0; k < NrClienti; k++){
 	    			IloExpr sumvincolo5a(GapEnv);
 	    			IloExpr sumvincolo5b(GapEnv);
 	    			for(IloInt i = 0; i < NrClienti + 1;i++){
 	    				sumvincolo5a += w[i][k][v]; 	
 	    			
 	    			}
 	    			for(IloInt l = 0; l < NrClienti; l++){
 	    				sumvincolo5b += w[k][l][v];
 	    			}
 	    			sprintf(str, "constr5(%li,%li)",v,k);
 	    			GapMod.add(sumvincolo5a >= sumvincolo5b).setName(str);
 	    		}
 	    	
 	    	}
                
               
                //6
              	//forall(v in Veicoli, <i, j> in Grafo)
 		//sum(k in Clienti)(y[<i,j>][v][k]) <= CapienzaVeicoli[v]*x[<i,j>][v];
		
		for(IloInt v = 0; v < NrVeicoli; v++){
			for(IloInt i = 0; i < NrClienti+1; i++){
				for(IloInt j = 0; j < NrClienti; j++ ){
					IloExpr sum6(GapEnv);
					IloExpr sum6a(GapEnv);
					for(IloInt k = 0; k < NrClienti;k++){
						sum6 += x[i][j][v][k];
						
					}
					sum6a = CapienzaVeicoli[v]*w[i][j][v];
					sprintf(str, "constr6(%li,%li,%li)",v,i,j);
					GapMod.add(sum6 <= sum6a).setName(str);
				
				}
			
			
			}
		
		
		}
		
   		 
		ofstream OutFile("cplex.log", ios::out);
		// construct CPLEX objects  - - - - - - - - - - - - - - - - - - - - - - -
		//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

		IloCplex cplex(GapMod);
		cplex.setOut(OutFile);

		// export the GAP model and the separation algorithms subproblems - - - -
		//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

		cplex.setParam(IloCplex::MIPDisplay, 2);
		cplex.exportModel("MipGap.lp");

		// solve GAP problem  as a MILP   - - - - - - - - - - - - - - - - - - - -
		//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

		cplex.solve();

		IloCplex::CplexStatus status = cplex.getCplexStatus();

		IloNum val;

		OutFile << endl << "MIP - Best Bound = " << cplex.getBestObjValue() << endl;
		
		
		IloInt count = 0;
		if( status != IloCplex::NodeLimInfeas ) {
			OutFile << "MIP - Best Integer = " << cplex.getObjValue() << endl;
			OutFile << "MIP solution : " << endl;
			OutFile << "\nflow var:\n" << endl;
			for( IloInt v = 0; v < NrVeicoli; v++ ) {
				for( IloInt j = 0; j < NrClienti; j++ ) {
					for(IloInt k = 0; k < NrClienti; k++){
						for( IloInt i = 0; i < NrClienti + 1; i++ ){
						
							val =  cplex.getValue(x[i][j][v][k]);
							if( val > 0 ){
								
								count = i +j;
								OutFile << "x[" << i << "," << j+1 <<  "," << v << "," << k <<"] = " << val << endl;
							}
						}
					
					}
					
				}
				
			}
			
		}
		
		if( status != IloCplex::NodeLimInfeas ) {
			OutFile << "\nrouting var:\n" << endl;
			for(IloInt j = 0; j < NrClienti;j++){
				for( IloInt i = 0; i < NrClienti + 1; i++ ){
					for( IloInt v = 0; v < NrVeicoli; v++ ) {
						val =  cplex.getValue(w[i][j][v]);
						if( val > 0 ){
							count = i +j;
							OutFile << "w[" << i << "," << j+1 <<  "," << v <<"] = " << val << endl;
						}
					}
					
				}
					
			}
				
		}
			
		

		OutFile.close();

		// deallocate CPLEX structures  - - - - - - - - - - - - - - -  - - - - - -

		GapEnv.end();
	

	
		
	}
	catch (IloException& ex) {
		cerr << "Error: " << ex << endl;
	}
	catch (...) {
		cerr << "Error" << endl;
	}

	return 0;
}
