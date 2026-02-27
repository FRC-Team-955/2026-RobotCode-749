package frc.robot.subsystems.shootersim;

import java.util.ArrayList;

public class PolyRegression {

    private ArrayList<Double> solveGaussian(int n, ArrayList<ArrayList<Double>> a) {
        for (int i = 0; i < n; ++i) {
            int piv = i;
            for (int j = i + 1; j < n; ++j)
                if (Math.abs(a[j][i]) > Math.abs(a[piv][i])) piv = j;
            std::swap(a[i], a[piv]);
            double d = a[i][i]; assert(std::fabs(d) > 1e-12);
            for (int j = i; j <= n; ++j) a[i][j] /= d;
            for (int k = i + 1; k < n; ++k) {
                double f = a[k][i];
                for (int j = i; j <= n; ++j) a[k][j] -= f * a[i][j];
            }
        }
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            double sum = a[i][n];
            for (int j = i + 1; j < n; ++j) sum -= a[i][j] * x[j];
            x[i] = sum / a[i][i];
        }
        return x;
    }
}
