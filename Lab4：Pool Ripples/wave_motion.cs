using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;

	Vector3 	block_v = Vector3.zero;
	Vector3 	block_w = Vector3.zero;

	Matrix4x4 I_ref;
	float mass;
	float dt = 0.1f;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}

		GameObject cube = GameObject.Find("Cube");
		Vector3[] vertices = mesh.vertices;
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

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
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

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(ref float[,] old_h, ref float[,] h, ref float [,] new_h, Vector3[] X)
	{		
		//Step 1:
		//Compute new_h based on the shallow wave model.
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{
			float tmp = -4.0f*h[i, j];
			if(i>=1) tmp += h[i-1, j];
			else tmp += h[i, j];
			if(j>=1) tmp += h[i, j-1];
			else tmp += h[i, j];
			if(i<size-1) tmp += h[i+1, j];
			else tmp += h[i, j];
			if(j<size-1) tmp += h[i, j+1];
			else tmp += h[i, j];
			new_h[i, j] = h[i,j]+(h[i,j]-old_h[i,j])*damping+tmp*rate;
		}

		//Step 2: Block->Water coupling
		GameObject cube = GameObject.Find("Cube");
		Bounds bounds_1 = cube.GetComponent<Collider>().bounds;
		Vector3 min_1 = bounds_1.min;
		Vector3 max_1 = bounds_1.max;
		int li_1 = (int)((min_1.x+size*0.05f)/0.1f);
		int ui_1 = (int)((max_1.x+size*0.05f)/0.1f);
		int lj_1 = (int)((min_1.z+size*0.05f)/0.1f);
		int uj_1 = (int)((max_1.z+size*0.05f)/0.1f);

		if(li_1<0)li_1 = 0;
		if(li_1>=100)li_1 = 99;
		if(ui_1<0)ui_1 = 0;
		if(ui_1>=100)ui_1 = 99;
		if(lj_1<0)lj_1 = 0;
		if(lj_1>=100)lj_1 = 99;
		if(uj_1<0)uj_1 = 0;
		if(uj_1>=100)uj_1 = 99;

		//for block 1, calculate low_h.
		//then set up b and cg_mask for conjugate gradient.
		Vector3 cube_x = cube.transform.position;
		Matrix4x4 R_1 = Matrix4x4.Rotate(cube.transform.rotation);
		Vector3 tao_1 = new Vector3(0.0f, 0.0f, 0.0f);
		float f_1 = 0.0f;

		for(int i=li_1; i<=ui_1; i++)
		for(int j=lj_1; j<=uj_1; j++)
		{
			Ray ray = new Ray(new Vector3(X[i*size+j].x, -1.0f, X[i*size+j].z), new Vector3(0, 1, 0));
			RaycastHit hit;
			if(Physics.Raycast(ray, out hit, Mathf.Infinity)){
				low_h[i, j] = hit.point.y;
				cg_mask[i, j] = true;
				b[i, j] = (new_h[i, j]-low_h[i,j])/rate;
				if(new_h[i, j]>=hit.point.y){
					f_1 += vh[i, j];
					Vector3 Rr = cube_x - hit.point;
					tao_1 += Rr*vh[i, j];
				}
			}
			else{
				vh[i, j] = 0.0f;
				cg_mask[i, j] = false;
			}
		}

		//Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li_1, ui_1, lj_1, uj_1);

		//for block 2, calculate low_h.
		GameObject block = GameObject.Find("Block");
		Bounds bounds_2 = block.GetComponent<Collider>().bounds;
		Vector3 min_2 = bounds_2.min;
		Vector3 max_2 = bounds_2.max;
		int li_2 = (int)((min_2.x+size*0.05f)/0.1f);
		int ui_2 = (int)((max_2.x+size*0.05f)/0.1f);
		int lj_2 = (int)((min_2.z+size*0.05f)/0.1f);
		int uj_2 = (int)((max_2.z+size*0.05f)/0.1f);

		if(li_2<0)li_2 = 0;
		if(li_2>=100)li_2 = 99;
		if(ui_2<0)ui_2 = 0;
		if(ui_2>=100)ui_2 = 99;
		if(lj_2<0)lj_2 = 0;
		if(lj_2>=100)lj_2 = 99;
		if(uj_2<0)uj_2 = 0;
		if(uj_2>=100)uj_2 = 99;

		//then set up b and cg_mask for conjugate gradient.
		Vector3 block_x = block.transform.position;
		Matrix4x4 R_2 = Matrix4x4.Rotate(block.transform.rotation);
		Vector3 tao_2 = new Vector3(0.0f, 0.0f, 0.0f);
		float f_2 = 0.0f;

		for(int i=li_2; i<=ui_2; i++)
		for(int j=lj_2; j<=uj_2; j++)
		{
			Ray ray = new Ray(new Vector3(X[i*size+j].x, -1.0f, X[i*size+j].z), new Vector3(0, 1, 0));
			RaycastHit hit;
			if(Physics.Raycast(ray, out hit, Mathf.Infinity)){
				low_h[i, j] = hit.point.y;
				cg_mask[i, j] = true;
				b[i, j] = (new_h[i, j]-low_h[i,j])/rate;
				if(new_h[i, j]>=hit.point.y){
					f_2 += vh[i, j];
					Vector3 Rr = block_x - hit.point;
					tao_2 += Rr*vh[i, j];
				}
			}
			else{
				vh[i, j] = 0.0f;
				cg_mask[i, j] = false;
			}
		}

		//Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li_2, ui_2, lj_2, uj_2);

		//Diminish vh.
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{
			vh[i, j] *= gamma;
		}

		//Update new_h by vh.
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{	
			float tmp = -4.0f*vh[i, j];
			if(i>=1) tmp += vh[i-1, j];
			else tmp += vh[i, j];
			if(j>=1) tmp += vh[i, j-1];
			else tmp += vh[i, j];
			if(i<size-1) tmp += vh[i+1, j];
			else tmp += vh[i, j];
			if(j<size-1) tmp += vh[i, j+1];
			else tmp += vh[i, j];
			new_h[i, j] = new_h[i,j]+tmp*rate;
		}

		//Step 3
		//old_h <- h; h <- new_h;
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{	
			old_h[i, j] = h[i, j];
			h[i, j] = new_h[i, j];
		}

		//Step 4: Water->Block coupling.
		//More TODO here.
		cube_v *= damping;
		cube_v += dt / mass * new Vector3(0.0f, -9.8f+f_1, 0.0f);
		cube_x += dt*cube_v;
		cube.transform.position = cube_x;

		Quaternion cube_q = cube.transform.rotation;
		Vector3 dw_1 = (R_1 * I_ref * R_1.transpose).inverse * tao_1;
		cube_w *= damping;
		cube_w += dt * dw_1;
		Quaternion Q_w_1 = new Quaternion(dt*cube_w[0]/2, dt*cube_w[1]/2, dt*cube_w[2]/2, 0);
		cube_q = Q_SUM_Q(cube_q, Q_MUL_Q(cube_q, Q_w_1));
		cube.transform.rotation = cube_q;

		block_v *= damping;
		block_v += dt / mass * new Vector3(0.0f, -9.8f+f_2, 0.0f);
		block_x += dt*cube_v;
		block.transform.position = block_x;

		Quaternion block_q = block.transform.rotation;
		Vector3 dw_2 = (R_2 * I_ref * R_2.transpose).inverse * tao_2;
		block_w *= damping;
		block_w += dt * dw_2;
		Quaternion Q_w_2 = new Quaternion(dt*block_w[0]/2, dt*block_w[1]/2, dt*block_w[2]/2, 0);
		block_q = Q_SUM_Q(block_q, Q_MUL_Q(block_q, Q_w_2));
		block.transform.rotation = block_q;

	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//Load X.y into h.
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{
			h[i, j] = X[i*size+j].y;
		}

		if (Input.GetKeyDown ("r")) 
		{
			//Add random water.
			int i = Random.Range(0, size);
			int j = Random.Range(0, size);
			float r = Random.Range(0f, 0.2f);
			h[i, j] += r;
			if(i>=1) h[i-1, j] -= r/8;
			if(i<size-1) h[i+1, j] -= r/8;
			if(j>=1) h[i, j-1] -= r/8;
			if(j<size-1) h[i, j+1] -= r/8;
			if(i>=1&&j>=1) h[i-1, j-1] -= r/8;
			if(i<size-1&&j<size-1) h[i+1, j+1] -= r/8;
			if(i>=1&&j<size-1) h[i-1, j+1] -= r/8;
			if(j>=1&&i<size-1) h[i+1, j-1] -= r/8;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(ref old_h, ref h, ref new_h, X);
		}

		//Store h back into X.y and recalculate normal.
		for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
		{
			X[i*size+j].y = h[i, j];
		}
		mesh.vertices = X;
		mesh.RecalculateNormals ();
	}
}
