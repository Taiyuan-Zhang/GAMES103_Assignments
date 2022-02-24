using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision
	Vector3 g = new Vector3(0, -9.8f, 0);		// for gravity 
	float uT = 0.5f; 							// for bounce coefficient
	
	Vector3[] vertices;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Quaternion Q_MUL_Q(Quaternion a, Quaternion b){
		Quaternion ret = new Quaternion(0, 0, 0, 0);
		ret.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
		ret.x = a.w*b.x + a.x*b.w + a.z*b.y - a.y*b.z;
		ret.y = a.w*b.y + a.y*b.w + a.x*b.z - a.z*b.x;
		ret.z = a.w*b.z + a.z*b.w + a.y*b.x - a.x*b.y;
		return ret;
	}

	Quaternion Q_SUM_Q(Quaternion a, Quaternion b){
		return new Quaternion(a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3]);
	}

	float Dot_Product(Vector3 a, Vector3 b){
		return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
	}

	Matrix4x4 Num_MUL_M(float num, Matrix4x4 a){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				ret[i, j] = a[i, j] * num;
			}
		}
		return ret;
	}

	Matrix4x4 M_SUB_M(Matrix4x4 a, Matrix4x4 b){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				ret[i, j] = a[i, j] - b[i, j];
			}
		}
		return ret;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		int cnt = 0;
		Vector3 total_V = new Vector3(0, 0, 0);
		Vector3 total_W = new Vector3(0, 0, 0);
		Vector3 x = transform.position;
		for(int i=0; i<vertices.Length; i++){
			Vector3 Rr_i = R * vertices[i];
			Vector3 x_i = x + Rr_i;
			if(Dot_Product(x_i-P, N)<0){
				Vector3 w_cross_Rr_i = Get_Cross_Matrix(w)*Rr_i;
				Vector3 v_i = v + w_cross_Rr_i;
				if(Dot_Product(v_i, N)<0){
					total_V += v_i;
					total_W += Rr_i;
					cnt += 1;
				}
			}
		}
		if(cnt>0){
			Vector3 V = total_V / cnt;
			Vector3 Rr = total_W / cnt;

			Vector3 v_N = Dot_Product(V, N) * N;
			Vector3 v_T = V - v_N;
			float a = Mathf.Max(1-uT*(1+restitution)*v_N.magnitude/v_T.magnitude, 0);
			Vector3 v_N_new = -restitution*v_N;
			Vector3 v_T_new = a * v_T;
			Vector3 v_new = v_N_new + v_T_new;

			Matrix4x4 I = Matrix4x4.identity;
			Matrix4x4 Rr_cross = Get_Cross_Matrix(Rr);
			Matrix4x4 K = M_SUB_M(Num_MUL_M(1/mass, I), Rr_cross*I_ref.inverse*Rr_cross);
			Vector3 j = K.inverse * (v_new - V);

			v = v + 1 / mass * j;
			Vector3 dw = I_ref.inverse * Rr_cross * j;
			w = w + dw;
			restitution *= 0.5f;
		}

	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (0.5f, 0, 0);
			launched=true;
		}

		if(launched==true){
			// Part I: Update velocities
			v = linear_decay * (v + dt * g);
			w = angular_decay * w;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x    = transform.position;
			x = x + v * dt;

			//Update angular status
			Quaternion q = transform.rotation;
			Quaternion Q_w = new Quaternion(dt*w[0]/2, dt*w[1]/2, dt*w[2]/2, 0);
			q = Q_SUM_Q(q, Q_MUL_Q(q, Q_w));
	
			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}
