import com.pedropathing.pathgen.*;

public class Auto_clipintake_paths {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(6.750, 43.000, Point.CARTESIAN),
                            new Point(42.000, 26.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierLine(
                            new Point(42.000, 26.000, Point.CARTESIAN),
                            new Point(14.000, 23.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-135))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(14.000, 23.000, Point.CARTESIAN),
                            new Point(41.000, 16.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-30))
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierLine(
                            new Point(41.000, 16.000, Point.CARTESIAN),
                            new Point(14.000, 13.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-135))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierCurve(
                            new Point(14.000, 13.000, Point.CARTESIAN),
                            new Point(40.000, 20.000, Point.CARTESIAN),
                            new Point(45.000, 6.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-80))
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierLine(
                            new Point(45.000, 6.000, Point.CARTESIAN),
                            new Point(14.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(180))
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierLine(
                            new Point(14.000, 30.000, Point.CARTESIAN),
                            new Point(8.000, 30.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathChain line8 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.000, 30.000, Point.CARTESIAN),
                            new Point(40.000, 69.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
}
