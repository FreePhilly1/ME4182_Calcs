void Kinematics::forceSolverPott(float Fx, float Fy, const float (&W)[2][4],
                                 float (&tensions)[4], float midTension) {
  float WT[4][2];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 2; ++j) {
      WT[i][j] = W[j][i];
    }
  }

  // t = tm + pinv(W)*(f - W * tm) = tm + W'.inv(W.W').(f - W.tm)
  float tm[4] = {midTension, midTension, midTension, midTension};
  // f - W.tm
  float Wtm[2];
  matmul(W, tm, Wtm);
  Fx -= Wtm[0];
  Fy -= Wtm[1];

  // inv(W.W')
  float WWT[2][2];  // 0-initialize
  matmul(W, WT, WWT);
  float WWTinv[2][2];
  inv2x2(WWT, WWTinv);
  // inv(W.W') * (f - W * tm)
  float intermediate[2];
  float fWtm[2] = {Fx, Fy};
  matmul(WWTinv, fWtm, intermediate);
  // tm + W'.inv(W'.W).(f - W'.tm)
  float intermediate2[4];
  matmul(WT, intermediate, intermediate2);
  matadd(tm, intermediate2, tensions);
}