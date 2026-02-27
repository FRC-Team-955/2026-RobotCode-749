package frc.robot.subsystems.shootersim;

import java.util.ArrayList;
import java.util.Collections;

public class PolyRegression {


    private static ArrayList<Double> solveGaussian(int n, ArrayList<ArrayList<Double>> a) {

        // Forward elimination (Gaussian elimination with partial pivoting)
        for (int i = 0; i < n; ++i) {

            // Find pivot row
            int piv = i;
            for (int j = i + 1; j < n; ++j) {
                if (Math.abs(a.get(j).get(i)) > Math.abs(a.get(piv).get(i))) {
                    piv = j;
                }
            }

            // Swap rows
            Collections.swap(a, i, piv);

            double d = a.get(i).get(i);
            if (Math.abs(d) < 1e-12) {
                return new ArrayList<Double>();
            }

            // Normalize pivot row
            for (int j = i; j <= n; ++j) {
                a.get(i).set(j, a.get(i).get(j) / d);
            }

            // Eliminate below
            for (int k = i + 1; k < n; ++k) {
                double f = a.get(k).get(i);
                for (int j = i; j <= n; ++j) {
                    a.get(k).set(j,
                            a.get(k).get(j) - f * a.get(i).get(j));
                }
            }
        }

        // Back substitution
        ArrayList<Double> x = new ArrayList<>(Collections.nCopies(n, 0.0));

        for (int i = n - 1; i >= 0; --i) {
            double sum = a.get(i).get(n);
            for (int j = i + 1; j < n; ++j) {
                sum -= a.get(i).get(j) * x.get(j);
            }
            x.set(i, sum / a.get(i).get(i));
        }

        return x;
    }

    public static ArrayList<Double> polyRegression(
            ArrayList<Double> x,
            ArrayList<Double> y,
            int degree) {

        int n = x.size();
        int m = degree + 1;

        // Compute sums of powers of x
        ArrayList<Double> sumPow = new ArrayList<>(
                Collections.nCopies(2 * degree + 1, 0.0));

        for (int p = 0; p <= 2 * degree; ++p) {
            double sum = 0.0;
            for (int i = 0; i < n; ++i) {
                sum += Math.pow(x.get(i), p);
            }
            sumPow.set(p, sum);
        }

        // Build augmented matrix A (m x (m+1))
        ArrayList<ArrayList<Double>> A = new ArrayList<>();

        for (int i = 0; i < m; ++i) {
            ArrayList<Double> row = new ArrayList<>(
                    Collections.nCopies(m + 1, 0.0));

            // Fill left-hand side (normal equation matrix)
            for (int j = 0; j < m; ++j) {
                row.set(j, sumPow.get(i + j));
            }

            // Fill right-hand side
            double rhs = 0.0;
            for (int k = 0; k < n; ++k) {
                rhs += Math.pow(x.get(k), i) * y.get(k);
            }
            row.set(m, rhs);

            A.add(row);
        }

        // Solve [A]c = b using Gaussian elimination
        return solveGaussian(m, A);
    }
}
