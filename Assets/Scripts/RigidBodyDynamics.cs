using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyDynamics : MonoBehaviour
{

	bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

	public float linear_decay = 0.999f;                // for velocity decay
    public float angular_decay = 0.98f;
    float restitution = 0.5f;                 // for collision
    float uT = 0.5f;
    Vector3 gravity = new Vector3(0, -10, 0); //for gravity
    Vector3[] vertices;

    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

	    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }
    Matrix4x4 Get_Sum_Matrix(Matrix4x4 a, Matrix4x4 b)
    {
        //4x4矩阵相加
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = a[0, 0] + b[0, 0];
        A[0, 1] = a[0, 1] + b[0, 1];
        A[0, 2] = a[0, 2] + b[0, 2];
        A[0, 3] = a[0, 3] + b[0, 3];
        A[1, 0] = a[1, 0] + b[1, 0];
        A[1, 1] = a[1, 1] + b[1, 1];
        A[1, 2] = a[1, 2] + b[1, 2];
        A[1, 3] = a[1, 3] + b[1, 3];
        A[2, 0] = a[2, 0] + b[2, 0];
        A[2, 1] = a[2, 1] + b[2, 1];
        A[2, 2] = a[2, 2] + b[2, 2];
        A[2, 3] = a[2, 3] + b[2, 3];
        A[3, 0] = a[3, 0] + b[3, 0];
        A[3, 1] = a[3, 1] + b[3, 1];
        A[3, 2] = a[3, 2] + b[3, 2];
        A[3, 3] = a[3, 3] + b[3, 3];
        return A;
    }

    float Dot_Product(Vector3 a, Vector3 b)
    {
        //3x3向量点积
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    Matrix4x4 Dot_Number(float num, Matrix4x4 a)
    {
        //4x4矩阵的数乘
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = a[0, 0] * num;
        A[0, 1] = a[0, 1] * num;
        A[0, 2] = a[0, 2] * num;
        A[0, 3] = a[0, 3] * num;
        A[1, 0] = a[1, 0] * num;
        A[1, 1] = a[1, 1] * num;
        A[1, 2] = a[1, 2] * num;
        A[1, 3] = a[1, 3] * num;
        A[2, 0] = a[2, 0] * num;
        A[2, 1] = a[2, 1] * num;
        A[2, 2] = a[2, 2] * num;
        A[2, 3] = a[2, 3] * num;
        A[3, 0] = a[3, 0] * num;
        A[3, 1] = a[3, 1] * num;
        A[3, 2] = a[3, 2] * num;
        A[3, 3] = a[3, 3] * num;
        return A;
    }

    Matrix4x4 Sub(Matrix4x4 a, Matrix4x4 b)
    {
        //4x4矩阵相减(其实也可以定义矩阵取负，再算相加)
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = a[0, 0] - b[0, 0];
        A[0, 1] = a[0, 1] - b[0, 1];
        A[0, 2] = a[0, 2] - b[0, 2];
        A[0, 3] = a[0, 3] - b[0, 3];
        A[1, 0] = a[1, 0] - b[1, 0];
        A[1, 1] = a[1, 1] - b[1, 1];
        A[1, 2] = a[1, 2] - b[1, 2];
        A[1, 3] = a[1, 3] - b[1, 3];
        A[2, 0] = a[2, 0] - b[2, 0];
        A[2, 1] = a[2, 1] - b[2, 1];
        A[2, 2] = a[2, 2] - b[2, 2];
        A[2, 3] = a[2, 3] - b[2, 3];
        A[3, 0] = a[3, 0] - b[3, 0];
        A[3, 1] = a[3, 1] - b[3, 1];
        A[3, 2] = a[3, 2] - b[3, 2];
        A[3, 3] = a[3, 3] - b[3, 3];
        return A;
    }

	// In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Matrix4x4 R0 = Matrix4x4.Rotate(transform.rotation);

        int count = 0;
        Vector3 avgV = new Vector3(0, 0, 0);
        Vector3 avgP = new Vector3(0, 0, 0);
        Vector3 x = transform.position;
        for (int i = 0; i < vertices.Length; i++)
        {
            //判断位置
            Vector3 Rri = R0 * vertices[i];
            Vector3 xi = x + Rri;
            Vector3 dis = xi - P;
            float d = Dot_Product(dis, N);
            if (d < 0)
            {
                //距离函数为负，判断法向速度
                Vector3 wxRi = Get_Cross_Matrix(w) * Rri;
                Vector3 vi = v + wxRi;
                if (Dot_Product(vi, N) < 0)
                {
                    avgP += Rri;
                    avgV += vi;
                    count++;
                }
            }
        }
        if (count > 0)
        {
            //碰撞点平均速度
            avgV = avgV / count;
            //碰撞点相对质心平均位移
            avgP = avgP / count;
            //碰撞前法向和切向速度
            Vector3 vN = Dot_Product(avgV, N) * N;
            Vector3 vT = avgV - vN;
            //碰撞后速度
            Vector3 vNnew = -restitution * vN;
            //切向速度的衰减比例
            float a = Mathf.Max(0, 1 - uT * (1 + restitution) * vN.magnitude / vT.magnitude);
            Vector3 vTnew = a * vT;
            Vector3 vnew = vNnew + vTnew;
            //求解冲量j
            Matrix4x4 I = Matrix4x4.identity;
            Matrix4x4 CRri = Get_Cross_Matrix(avgP);
            Matrix4x4 Iinverse = I_ref.inverse;
            float mInverse = 1 / mass;
            Matrix4x4 K = Sub(Dot_Number(mInverse, I), CRri * Iinverse * CRri);
            Vector3 j = K.inverse * (vnew - avgV);
            //更新v和w
            v = v + mInverse * j;
            Vector3 dw = Iinverse * CRri * j;
            w = w + dw;
            restitution *= 0.5f;
        }
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(5, 2, 0);
            w = new Vector3(0.5f, 0, 0);
            launched = true;
        }
        if (launched == true)
        {
            // Part I: Update velocities
            v = linear_decay * (v + dt * gravity);
            //Matrix4x4 R0=Matrix4x4.Rotate(transform.rotation);
            w = angular_decay * w;
            // Part II: Collision Impulse
            //Collision_Impulse(new Vector3(0, -1.097432f, 0), new Vector3(0, 1, 0));
            //Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;
            x = x + v * dt;
            //Update angular status
            Quaternion q = transform.rotation;
            //做四元数的叉乘[s1s2-v1v2 s1v2+s2v1+v1xv2]
            //实部
            float sq = q.w;
            //虚部
            Vector3 vq = new Vector3(q.x, q.y, q.z);
            //q1=q0+[0,dt/2*w]xq0
            Vector3 vw = w * dt / 2;
            float sdq = -Dot_Product(vw, vq);
            Vector3 temp = Get_Cross_Matrix(vw) * vq;
            Vector3 svq = (sq * vw) + temp;
            Quaternion q1 = new Quaternion(vq[0] + svq[0], vq[1] + svq[1], vq[2] + svq[2], sq + sdq);

            transform.position = x;
            transform.rotation = q1;
        }        
    }
}
