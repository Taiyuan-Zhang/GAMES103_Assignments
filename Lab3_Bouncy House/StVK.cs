using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class StVK : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;
	Vector3 gravity     = new Vector3(0.0f, -9.8f, 0.0f);
	float restitution 	= 0.5f;
	float uT 			= 0.5f;
	Vector3 floor;
	Vector3 floor_N 	= new Vector3(0, 1, 0);

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];
		floor = GameObject.Find("Floor").transform.position;

		//Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for(int tet=0; tet<tet_number; tet++){
			inv_Dm[tet]=Build_Edge_Matrix(tet).inverse;
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//Need to build edge matrix here.
		Vector3 x_0 = X[Tet[tet*4+0]];
		Vector3 x_1 = X[Tet[tet*4+1]];
		Vector3 x_2 = X[Tet[tet*4+2]];
		Vector3 x_3 = X[Tet[tet*4+3]];
		Vector3 x_10 = x_1-x_0;
		Vector3 x_20 = x_2-x_0;
		Vector3 x_30 = x_3-x_0;
		ret[0, 0] = x_10[0];
		ret[1, 0] = x_10[1];
		ret[2, 0] = x_10[2];
		ret[0, 1] = x_20[0];
		ret[1, 1] = x_20[1];
		ret[2, 1] = x_20[2];
		ret[0, 2] = x_30[0];
		ret[1, 2] = x_30[1];
		ret[2, 2] = x_30[2];
		ret[3, 3] = 1;

		return ret;
    }

	Matrix4x4 Num_MUL_M(float num, Matrix4x4 a){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				ret[i, j] = a[i, j] * num;
			}
		}
		ret[3,3]=1;
		return ret;
	}

	Matrix4x4 M_ADD_M(Matrix4x4 a, Matrix4x4 b){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				ret[i, j] = a[i, j] + b[i, j];
			}
		}
		ret[3,3]=1;
		return ret;
	}

	Matrix4x4 M_SUB_M(Matrix4x4 a, Matrix4x4 b){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				ret[i, j] = a[i, j] - b[i, j];
			}
		}
		ret[3,3]=1;
		return ret;
	}

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0; i<number; i++)
    	{
    		//Add gravity to Force.
			V[i] += dt * gravity;
			V_sum[i] = new Vector3(0, 0, 0);
			V_num[i] = 0;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//Deformation Gradient
			Matrix4x4 Dm_inv = inv_Dm[tet];
			Matrix4x4 F = Build_Edge_Matrix(tet) * Dm_inv;
    		
    		//SVD
			Matrix4x4 U = Matrix4x4.identity;
            Matrix4x4 A = Matrix4x4.identity;
            Matrix4x4 V = Matrix4x4.identity;
            svd.svd(F, ref U, ref A, ref V);
            Matrix4x4 V_T = V.transpose;

    		//Second PK Stress
			Matrix4x4 diag = Matrix4x4.identity;
            float Ic = A[0,0]*A[0,0]+A[1,1]*A[1,1]+A[2,2]*A[2,2];
            diag[0, 0] = 0.5f*A[0,0]*(stiffness_0*(Ic-3.0f))+stiffness_1*(A[0,0]*A[0,0]*A[0,0]-A[0,0]);
            diag[1, 1] = 0.5f*A[1,1]*(stiffness_0*(Ic-3.0f))+stiffness_1*(A[1,1]*A[1,1]*A[1,1]-A[1,1]);
            diag[2, 2] = 0.5f*A[2,2]*(stiffness_0*(Ic-3.0f))+stiffness_1*(A[2,2]*A[2,2]*A[2,2]-A[2,2]);
            Matrix4x4 P = U*diag*V_T;

    		//Elastic Force
			Matrix4x4 f = Num_MUL_M(-1.0f/6.0f/Dm_inv.determinant,P*Dm_inv.transpose);

			Vector3 f1 = new Vector3(f[0,0], f[1,0],f[2,0]);
			Vector3 f2 = new Vector3(f[0,1], f[1,1],f[2,1]);
			Vector3 f3 = new Vector3(f[0,2], f[1,2],f[2,2]);
			Vector3 f0 = -f1-f2-f3;

			Force[Tet[tet*4+0]] += f0;
			Force[Tet[tet*4+1]] += f1;
			Force[Tet[tet*4+2]] += f2;
			Force[Tet[tet*4+3]] += f3;
    	}

    	for(int i=0; i<number; i++)
    	{
    		//Update X and V here.
			V[i] = damp*(V[i] + dt * Force[i] / mass);
    	}

		//Laplacian smoothing
		for(int tet=0; tet<tet_number; tet++){
			Vector3 v_0 = V[Tet[tet*4+0]];
			Vector3 v_1 = V[Tet[tet*4+1]];
			Vector3 v_2 = V[Tet[tet*4+2]];
			Vector3 v_3 = V[Tet[tet*4+3]];
			V_sum[Tet[tet*4+0]] += (v_1+v_2+v_3)/3;
			V_sum[Tet[tet*4+1]] += (v_0+v_2+v_3)/3;
			V_sum[Tet[tet*4+2]] += (v_0+v_1+v_3)/3;
			V_sum[Tet[tet*4+3]] += (v_0+v_1+v_2)/3;
			V_num[Tet[tet*4+0]] += 1;
			V_num[Tet[tet*4+1]] += 1;
			V_num[Tet[tet*4+2]] += 1;
			V_num[Tet[tet*4+3]] += 1;
		}
		for(int i=0; i<number; i++){
			V[i] = (V_sum[i]/V_num[i] + V[i])/2;
			X[i] += dt * V[i];
			V_sum[i] = new Vector3(0, 0, 0);
			V_num[i] = 0;
			// (Particle) collision with floor.
			if(Vector3.Dot(X[i]-floor, floor_N)<0.0f){
				if(Vector3.Dot(V[i], floor_N)<0){
					Vector3 v_N = Vector3.Dot(V[i], floor_N)*floor_N;
					Vector3 v_T = V[i] - v_N;
					float a = Mathf.Max(1-uT*(1+restitution)*v_N.magnitude/v_T.magnitude, 0);
					Vector3 v_N_new = -restitution*v_N;
					Vector3 v_T_new = a * v_T;
					V[i] = v_N_new + v_T_new;
					X[i] = X[i] - Vector3.Dot(X[i]-floor, floor_N)*floor_N;
				}
			}
			Force[i] = new Vector3(0, 0, 0);
		}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
