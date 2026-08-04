//package org.firstinspires.ftc.teamcode.library.internal.math.mock;
//
//import static org.junit.jupiter.api.Assertions.assertEquals;
//
//import org.firstinspires.ftc.teamcode.library.internal.Pose2D;
//import org.junit.jupiter.api.Test;
//
//public class TestMockDT {
//    @Test
//    void testControl() {
//        // Test DT straight ahead
//        double angle = Math.toRadians(0);
//        MockDT dt = new MockDT(new Pose2D(0, 0, angle));
//
//        dt.control(1, 1, 0);
//
//        dt.update();
//
//        Pose2D position = dt.getPosition();
//
//        assertEquals(1.0, position.x, 0.01);
//        assertEquals(1.0, position.y, 0.01);
//        assertEquals(angle, position.h, 0.01);
//
//        // Test 90 deg rotation
//        angle = Math.toRadians(90);
//        dt = new MockDT(new Pose2D(0, 0, angle));
//
//        dt.control(1, 1, 0);
//
//        dt.update();
//
//        position = dt.getPosition();
//
//        assertEquals(-1.0, position.x, 0.01);
//        assertEquals(1.0, position.y, 0.01);
//        assertEquals(angle, position.h, 0.01);
//
//        // Test -90 deg rotation
//        angle = Math.toRadians(-90);
//        dt = new MockDT(new Pose2D(0, 0, angle));
//
//        dt.control(1, 1, 0);
//
//        dt.update();
//
//        position = dt.getPosition();
//
//        assertEquals(1.0, position.x, 0.01);
//        assertEquals(-1.0, position.y, 0.01);
//        assertEquals(angle, position.h, 0.01);
//    }
//}
