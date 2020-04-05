/* Author- Rishi Malhan. */ 

#ifndef Cholesky_Decomposition_H
#define Cholesky_Decomposition_H
#include <iostream>
#include <cmath>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class cholesky
{
    private:
        int dimension;
        Eigen::MatrixXd A;
        Eigen::MatrixXd L;
        Eigen::MatrixXd D;
    public:
        cholesky(int _dimension)
        {
            dimension = _dimension;
            L = Eigen::MatrixXd::Identity(dimension,dimension);
            D = Eigen::MatrixXd::Identity(dimension,dimension);
        }

        void ldl(Eigen::MatrixXd _A)
        {
        A = _A;
        int dimension = A.rows();
        for( int j=0; j<dimension; ++j )
            { // each columns
            L(j, j) = 1;
            D(j,j) = A(j,j);
            for( int k=0; k<=j-1; ++k)
            { // sum
                D(j,j) -= L(j,k) * L(j,k) * D(k,k);
            }
            for( int i=j+1; i<dimension; ++i )
                { // remaining rows
                L(i,j) = A(i,j);
                for( int k=0; k<=j-1; ++k)
                { // sum
                    L(i,j) -= L(i,k) * L(j,k) * D(k,k);
                }
                L(i,j) /= D(j,j);
                } // for i in rows
            }
        }

        Eigen::MatrixXd getL()
        {
            return L;
        }

        Eigen::MatrixXd getD()
        {
            return D;
        }
};

#endif