#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_

vector< vector< double > > makeVertices(double x, double y, int numVerts){
	vector< vector< double > > vertices(numVerts, vector<double>(2));

	srand (time(NULL));
	double vertx, verty;
	int xx = x;
	int yy = y;
	double testx, testy;

	for(int i = 0; i < numVerts; i++){
		if (i == 0)
		{
			vertices[i][0] = 0 ;
			vertices[i][1] = 0 ;
		}
		else if (i == numVerts-1)
		{
			vertices[i][0] = x ;
			vertices[i][1] = y ;
		}
		else
		{
			vertx = rand() % xx;
			verty = rand() % yy;
			vertices[i][0] = vertx ;
			vertices[i][1] = verty ;
		}
	}

	// Write vertices to txt file
//	stringstream vFileName ;
//	vFileName << "../results/vertices" << trialNum << ".txt" ;

//	ofstream vertsFile ;
//	vertsFile.open(vFileName.str().c_str()) ;

//	for (ULONG i = 0; i < vertices.size(); i++)
//	{
//		vertsFile << vertices[i][0] << "," << vertices[i][1] << "\n" ;
//	}
//	vertsFile.close() ;

	return vertices;
}

vector<double> linspace(double a, double b, int n)
{
	vector<double> array ;
	double step = (b-a)/(n-1) ;
	while (a<=b)
	{
		array.push_back(a) ;
		a += step ;
	}
	return array ;
}

bool ComputeImprovementProbability(Vertex * A, Vertex * B)
{
	vector<Node *> ANodes = A->GetNodes();
	vector<Node *> BNodes = B->GetNodes();
	
	double c_A0 = A->GetCTC() ;
	double c_B0 = B->GetCTC() ;
	double max_3sig = ANodes[0]->GetMeanCTG() + 3*ANodes[0]->GetVarCTG() ;
	double min_3sig = ANodes[0]->GetMeanCTG() - 3*ANodes[0]->GetVarCTG() ;
	for (int i = 0; i < ANodes.size(); i++)
	{
		if (max_3sig < ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG())
			max_3sig = ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG() ;
		if (min_3sig > ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG())
			min_3sig = ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG() ;
	}
	for (int i = 0; i < BNodes.size(); i++)
	{
		if (max_3sig < BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG())
			max_3sig = BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG() ;
		if (min_3sig > BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG())
			min_3sig = BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG() ;
	}
	
	int n = 10000 ;
	vector<double> x = linspace(min_3sig,max_3sig,n) ;
	double dx = x[1]-x[0] ;
	double pImprove = 0.0 ;
	for (int k = 0; k < x.size(); k++)
	{
		double p_cAi = 0.0 ;
		for (int i = 0; i < ANodes.size(); i++)
		{
			double mu_Ai = ANodes[i]->GetMeanCTG() ;
			double sig_Ai = ANodes[i]->GetVarCTG() ;
			double p_cA1 = (1/(sig_Ai*sqrt(2*pi)))*exp(-(pow(x[k]-mu_Ai,2))/(2*pow(sig_Ai,2))) ;
			double p_cA2 = 1.0 ;
			for (int j = 0; j < ANodes.size(); j++)
			{
				double mu_Aj = ANodes[j]->GetMeanCTG() ;
				double sig_Aj = ANodes[j]->GetVarCTG() ;
				if (j != i)
					p_cA2 *= 0.5*erfc((x[k]-mu_Aj)/(sig_Aj*sqrt(2))) ;
			}
			p_cAi += p_cA1*p_cA2 ;
		}
		double p_cBi = 1.0 ;
		for (int i = 0; i < BNodes.size(); i++)
		{
			double mu_Bi = BNodes[i]->GetMeanCTG() ;
			double sig_Bi = BNodes[i]->GetVarCTG() ;
			p_cBi *= 0.5*erfc((x[k]-(c_B0-c_A0)-mu_Bi)/(sig_Bi*sqrt(2))) ;
		}
		pImprove += (p_cAi)*(1-p_cBi)*dx ;
	}
	
	return (pImprove<=0.5);
}

double CalculateTrueEdgeCost(Vertex * v1, Vertex * v2, const sensor_msgs::LaserScan& msg){
  double trueCost = 0.0 ;
}

void SetTrueEdgeCosts(Vertex * v1, vector<Vertex *> v2, const sensor_msgs::LaserScan& msg)
{
	// Set cost-to-come of v2 vertices as true cost of edge traversal
	for (int i = 0; i < v2.size(); i++)
	{
	  double trueCost = CalculateTrueEdgeCost(v1,v2[i],msg) ;
		v2[i]->SetCTC(trueCost) ;
		break ;
	}
}
#endif
