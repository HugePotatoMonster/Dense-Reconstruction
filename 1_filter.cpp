// 方程
    // d_ref * f_ref = d_cur * ( R_RC * f_cur ) + t_RC
    // => [ f_ref^T f_ref, -f_ref^T f_cur ] [d_ref] = [f_ref^T t]
    //    [ f_cur^T f_ref, -f_cur^T f_cur ] [d_cur] = [f_cur^T t]
    // 二阶方程用克莱默法则求解并解之
Vector3d t = T_R_C.translation();
Vector3d f2 = T_R_C.rotation_matrix() * f_curr;
Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
double A[4];
A[0] = f_ref.dot(f_ref);
A[2] = f_ref.dot(f2);
A[1] = -A[2];
A[3] = -f2.dot(f2);
double d = A[0] * A[3] - A[1] * A[2];
Vector2d lambdavec =
Vector2d(A[3] * b(0, 0) - A[1] * b(1, 0),
    -A[2] * b(0, 0) + A[0] * b(1, 0)) / d;
Vector3d xm = lambdavec(0, 0) * f_ref;
Vector3d xn = t + lambdavec(1, 0) * f2;
Vector3d d_esti = (xm + xn) / 2.0;  // 三角化算得的深度向量
double depth_estimation = d_esti.norm();   // 深度值