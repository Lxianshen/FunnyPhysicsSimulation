using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	public float dt 			= 0.003f;
    public float mass 			= 1;
	public float stiffness_0	= 20000.0f;
    public float stiffness_1 	= 5000.0f;
    public float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	Vector3[]   V_sum;
	int[]		V_num;
	int number;

	Matrix4x4[] inv_Dm;

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
			//tet_number = tet_number - 10;
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
			//number = number - 10;
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

		inv_Dm  = new Matrix4x4[tet_number];
		for(int tet=0; tet<tet_number; tet++)
			inv_Dm[tet]=Build_Edge_Matrix(tet).inverse;
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
		ret[0, 0]=X[Tet[tet*4+1]].x-X[Tet[tet*4+0]].x;
		ret[1, 0]=X[Tet[tet*4+1]].y-X[Tet[tet*4+0]].y;
		ret[2, 0]=X[Tet[tet*4+1]].z-X[Tet[tet*4+0]].z;
		ret[3, 0]=0;

		ret[0, 1]=X[Tet[tet*4+2]].x-X[Tet[tet*4+0]].x;
		ret[1, 1]=X[Tet[tet*4+2]].y-X[Tet[tet*4+0]].y;
		ret[2, 1]=X[Tet[tet*4+2]].z-X[Tet[tet*4+0]].z;
		ret[3, 1]=0;

		ret[0, 2]=X[Tet[tet*4+3]].x-X[Tet[tet*4+0]].x;
		ret[1, 2]=X[Tet[tet*4+3]].y-X[Tet[tet*4+0]].y;
		ret[2, 2]=X[Tet[tet*4+3]].z-X[Tet[tet*4+0]].z;
		ret[3, 2]=0;

		ret[0, 3]=0;
		ret[1, 3]=0;
		ret[2, 3]=0;
		ret[3, 3]=1;
		return ret;
    }

    void Smooth_V()
    {
    	for(int i=0; i<number; i++)
    	{
    		V_sum[i]=new Vector3(0, 0, 0);
    		V_num[i]=0;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		Vector3 sum=V[Tet[tet*4+0]]+V[Tet[tet*4+1]]+V[Tet[tet*4+2]]+V[Tet[tet*4+3]];
    		V_sum[Tet[tet*4+0]]+=sum;
    		V_sum[Tet[tet*4+1]]+=sum;
    		V_sum[Tet[tet*4+2]]+=sum;
    		V_sum[Tet[tet*4+3]]+=sum;
			V_num[Tet[tet*4+0]]+=4;
    		V_num[Tet[tet*4+1]]+=4;
    		V_num[Tet[tet*4+2]]+=4;
    		V_num[Tet[tet*4+3]]+=4;
       	}

       	for(int i=0; i<number; i++)
       	{
       		V[i]=0.9f*V[i]+0.1f*V_sum[i]/V_num[i];
       	}
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	//Add Gravity.
    	for(int i=0 ;i<number; i++)
    	{
    		Force[i]=new Vector3(0, -9.8f*mass, 0);
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//Deformation Gradient
    		Matrix4x4 F=Build_Edge_Matrix(tet)*inv_Dm[tet];
    		Matrix4x4 U=Matrix4x4.zero;
    		Matrix4x4 D=Matrix4x4.zero;
    		Matrix4x4 V=Matrix4x4.zero;
    		Matrix4x4 P=Matrix4x4.zero;
			Matrix4x4 result;

			if(false)
    		{
    			svd.svd(F, ref U, ref D, ref V);
				float I		= D[0,0]*D[0,0]+D[1,1]*D[1,1]+D[2,2]*D[2,2];
	    		float J		= D[0,0]*D[1,1]*D[2,2];
	    		float II	= D[0,0]*D[0,0]*D[0,0]*D[0,0]+D[1,1]*D[1,1]*D[1,1]*D[1,1]+D[2,2]*D[2,2]*D[2,2]*D[2,2];
				float III	= J*J;
				float dEdI	= stiffness_0*(I-3)*0.25f-stiffness_1*0.5f;
				float dEdII	= stiffness_1*0.25f;
				float dEdIII= 0;

				P[0,0] = 2*dEdI*D[0,0]+4*dEdII*D[0,0]*D[0,0]*D[0,0]+2*dEdIII*III/D[0,0];
				P[1,1] = 2*dEdI*D[1,1]+4*dEdII*D[1,1]*D[1,1]*D[1,1]+2*dEdIII*III/D[1,1];
				P[2,2] = 2*dEdI*D[2,2]+4*dEdII*D[2,2]*D[2,2]*D[2,2]+2*dEdIII*III/D[2,2];

				result=U*P*V.transpose*inv_Dm[tet].transpose;
			}
			{
				Matrix4x4 G=F.transpose*F;
				G[0, 0]-=1;
				G[1, 1]-=1;
				G[2, 2]-=1;
				for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					G[i, j]*=0.5f;
				Matrix4x4 S=Matrix4x4.zero;
				for(int i=0; i<3; i++)
				for(int j=0; j<3; j++)
					S[i, j]=stiffness_1*2*G[i, j];
				S[0, 0]+=stiffness_0*(G[0,0]+G[1,1]+G[2,2]);
				S[1, 1]+=stiffness_0*(G[0,0]+G[1,1]+G[2,2]);
				S[2, 2]+=stiffness_0*(G[0,0]+G[1,1]+G[2,2]);

				//Debug.Log(F*S);
				//Debug.Log(U*P*V.transpose);
				//result=U*P*V.transpose*inv_Dm[tet].transpose;
				result=F*S*inv_Dm[tet].transpose;
			}

			float scale=-1/(inv_Dm[tet].determinant*6);

			Force[Tet[tet*4+0]].x-=scale*(result[0,0]+result[0,1]+result[0,2]);
			Force[Tet[tet*4+0]].y-=scale*(result[1,0]+result[1,1]+result[1,2]);
			Force[Tet[tet*4+0]].z-=scale*(result[2,0]+result[2,1]+result[2,2]);
			Force[Tet[tet*4+1]].x+=scale*result[0,0];
			Force[Tet[tet*4+1]].y+=scale*result[1,0];
			Force[Tet[tet*4+1]].z+=scale*result[2,0];
			Force[Tet[tet*4+2]].x+=scale*result[0,1];
			Force[Tet[tet*4+2]].y+=scale*result[1,1];
			Force[Tet[tet*4+2]].z+=scale*result[2,1];
			Force[Tet[tet*4+3]].x+=scale*result[0,2];
			Force[Tet[tet*4+3]].y+=scale*result[1,2];
			Force[Tet[tet*4+3]].z+=scale*result[2,2];
    	}

		Smooth_V();
    	for(int i=0; i<number; i++)
    	{
    		V[i]=(V[i]+dt*Force[i]/mass)*damp;
    		X[i]=X[i]+dt*V[i];

    		if(X[i].y<-3.0f)
    		{
    			V[i].x=0;
    			V[i].z=0;
    			V[i].y+=(-3.0f-X[i].y)/dt;
    			X[i].y =-3.0f;
    		}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	//for(int l=0; l<10; l++)
    	//	 _Update();
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
