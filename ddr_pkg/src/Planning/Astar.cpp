#include "ddr_pkg/planner.h"

void RRT_Astar::Astar_search(Tree& N)
{
	Tree OPEN;
	Tree T;
	
	Nodo N_best;
	Nodo Ns, Ng;
	
	ros::Rate r(10);
	
	trajectory.clear();
	
	for(int i=0; i<N.size(); i++)
	{
		if(N[i].id == 0)
		{
			N[i].visited = true;
			N[i].f_Ni = fCost(N[i]);
			N[i].parent = 0;
			Ns = N[i];
		}
			
		if(N[i].id == 200)
			Ng = N[i];
	}

	T.push_back(Ns);
	OPEN.push_back(Ns);
	
	int i;
	while(!OPEN.empty())
	{			
		find_extract_Nbest(N_best, OPEN);
		
		for(int s=0; s<T.size(); s++)
			//if(T[s].parent != 0)
				cout<<T[s].id<<endl;
		
		if(N_best.id == Ng.id)
		{
			cout<<"PERCORSO TROVATO"<<endl;
			break;
		}
		
		for(int m=0; m<N_best.adj_list.size(); m++)
		{
			for(int j=0; j<N.size(); j++)
			{
				if(N_best.adj_list[m] == N[j].id)
				{
					i = j;
					cout<<" Id vicino: "<<N_best.adj_list[m]<<endl;	
				}	
			}
			
			if(!N[i].visited)
			{
				for(int f=0; f<N.size(); f++)
					if(N[f].id == N_best.id)
						N[i].parent = &N[f];
				
				//N[i].parent = &N_best;
				N[i].visited = true;
				T.push_back(N[i]);
				OPEN.push_back(N[i]);
				cout<<"Ni: "<<N[i].id<<endl;
				cout<<"Visited: "<<N[i].visited<<endl;
				cout<<"Nbest: "<<N_best.id<<endl;
				cout<<"Ni parent: "<<N[i].parent->id<<endl;
				cout<<"g(Ni): "<<gCost(N[i])<<endl<<endl;
				cout<<"f(Ni): "<<fCost(N[i])<<endl<<endl;	
			}
			
			
			//float pm = gCost(N_best) + delta;
			//float sm = gCost(N[i]);
			else if(gCost(N_best) + delta < gCost(N[i]))
			{
				//redirect the pointer of N[i] in T toward N_best
				for(int k=0; k<T.size(); k++)
					if(T[k].id == N[i].id)
						for(int f=0; f<N.size(); f++)
							if(N[f].id == N_best.id)
								T[k].parent = &N[f];
				
				if(!in(N[i], OPEN))
					OPEN.push_back(N[i]);

				else
				{
					for(int k=0; k<T.size(); k++)
						if(T[k].id == N[i].id)
							T[k].f_Ni = fCost(N[i]);
							
					cout<<"Aggiornato"<<endl;
				}//fine else
				
			}//fine else if
	
		}//fine for
	
	}//fine while
	
	for(int j=0; j<T.size(); j++)
	{ 
		if(T[j].id == Ng.id)
		{
			Nodo* current_node = &T[j];
			while(current_node->id != Ns.id)
			{
				for(int c=0; c<T.size(); c++) //esploro l'albero T per memorizzare il percorso ottimo
					if(T[c].id == current_node->id)
						trajectory.push_back(T[c]);
				
				visualization_minimum_path(current_node, 0.0, 1.0, 0.0);
				current_node = current_node->parent;
				r.sleep();
			}
			
			trajectory.push_back(Ns);
		}	
	}		
	
}//fine Astar



void RRT_Astar::find_extract_Nbest(Nodo& Nb, Tree& open)
{
	//find and extract N best from OPEN
	double min_fCost;
	double curr_fCost;
	int index = 0;
	for(int i=0; i<open.size(); i++)
	{
		if(i == 0)
			min_fCost = open[i].f_Ni; 
		
		curr_fCost = open[i].f_Ni;
		if(curr_fCost < min_fCost)
		{
			min_fCost = curr_fCost;
			index = i;
		}			
	}
		Nb = open[index];
		open.erase(open.begin() + index);
}



float RRT_Astar::fCost(Nodo N)
{
	float f;
	
	if(N.id == 0)
		f = hCost(N);
	
	else
		f = gCost(N) + hCost(N);
		
	return f;
}


float RRT_Astar::gCost(const Nodo& N)
{
	float g;
	int cont = 0;
	bool found = false;
	const Nodo* curr_node = &N;

	while(!found)
	{
		curr_node = curr_node->parent;
		
		if(curr_node == 0)
			found = true;
			
		else
			cont++;
	}
	
	g = delta*cont;
	return g;
}



float RRT_Astar::hCost(Nodo N)
{
	float h;
	Point2D end;
	end.x = destination.x;//final_position.x;
	end.y = destination.y;//final_position.y;
	h = get_distance(N.position, end);
	return h;
}


bool RRT_Astar::in(Nodo N, Tree open)
{
	for(int i=0; i<open.size(); i++)
		if(open[i].id == N.id)
			return true;
	
	return false;
}

