package frc.robot.subsystems.shootersim;

import java.util.ArrayList;

public class InterpolationTable {
    private final ArrayList<Double> in = new ArrayList<>();
    private final ArrayList<Double> out = new ArrayList<>();

    public void add(double inPt, double outPt) {
        in.add(inPt);
        out.add(outPt);
    }

    public double getLinear(double inPt){
        double bestUnderDiff = Double.MAX_VALUE;
        double bestOverDiff = Double.MAX_VALUE;
        int bestUnderIndex  = -1;
        int bestOverIndex = -1;

        for (int i = 0; i < in.size(); i++) {
            double current = in.get(i);
            double diff = current - inPt;

            if (current <= inPt) { // UNDER
                double underDiff = inPt - current;
                if (underDiff < bestUnderDiff) {
                    bestUnderDiff = underDiff;
                    bestUnderIndex = i;
                }
            }
            if (current >= inPt) { // OVER
                double overDiff = current - inPt;
                if (overDiff < bestOverDiff) {
                    bestOverDiff = overDiff;
                    bestOverIndex = i;
                }
            }
        }

        // Exact match
        if (bestUnderIndex == bestOverIndex) {
            return out.get(bestUnderIndex);
        }

        // Edge safety
        if (bestUnderIndex == -1) return out.get(bestOverIndex);
        if (bestOverIndex == -1) return out.get(bestUnderIndex);

        double x1 = in.get(bestUnderIndex);
        double x2 = in.get(bestOverIndex);
        double y1 = out.get(bestUnderIndex);
        double y2 = out.get(bestOverIndex);

        // linear interpolation formula
        double interpolatedPoint =
                y1 + (y2 - y1) * (inPt - x1) / (x2 - x1);

        return interpolatedPoint;
    }
}
