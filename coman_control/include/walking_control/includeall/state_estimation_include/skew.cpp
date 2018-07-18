//Compute skew matrix from a vector

void skew(double vector[3], double skew_matrix[3][3])
{
    skew_matrix[0][0] = 0;
    skew_matrix[0][1] = -vector[2];
    skew_matrix[0][2] = vector[1];
    skew_matrix[1][0] = vector[2];
    skew_matrix[1][1] = 0;
    skew_matrix[1][2] = -vector[0];
    skew_matrix[2][0] = -vector[1];
    skew_matrix[2][1] = vector[0];
    skew_matrix[2][2] = 0;

}
