using System;
using System.Numerics;
using System.IO;
using System.Text.Json;
using System.Collections.Generic;


namespace Data_Collection;

public class Rod
{
    public float rotation;
    public float angular_velocity;
    public float rod_length;
    public float rod_mass;
    public Vector2 origin_pos;

    public void GiveValues(float rotation_init,
                    float clockwise_angular_velocity_init,
                    float rod_length_init,
                    float rod_weight_init,
                    Vector2 origin_pos_init)
    {
        rotation = rotation_init;
        angular_velocity = clockwise_angular_velocity_init;
        rod_length = rod_length_init;
        rod_mass = rod_weight_init;
        origin_pos = origin_pos_init;
    }
    public Vector2 FindRodEnd()
    {
        float rod_end_X = origin_pos.X - rod_length * (float)Math.Sin(rotation);
        float rod_end_Y = origin_pos.Y + rod_length * (float)Math.Cos(rotation);
        return new Vector2(rod_end_X, rod_end_Y);
    }
    private float Sin(float value)
    {
        return (float)Math.Sin(value);
    }
    private float Cos(float value)
    {
        return (float)Math.Cos(value);
    }

    private List<float> CalculateRungeKutta(float theta1, float theta2, float vel1, float vel2, float deltaTime, Rod other_rod)
    {
        // Uses the Runge-Kutta method to calculate the updated angles and velocities
        float k1theta1 = vel1;
        float k1vel1 = F1(theta1, theta2, vel1, vel2, other_rod);
        float k1theta2 = vel2;
        float k1vel2 = F2(theta1, theta2, vel1, vel2, other_rod);

        float theta1mid = theta1 + k1theta1 * deltaTime / 2;
        float vel1mid = vel1 + k1vel1 * deltaTime / 2;
        float theta2mid = theta2 + k1theta2 * deltaTime / 2;
        float vel2mid = vel2 + k1vel2 * deltaTime / 2;

        float k2theta1 = vel1mid;
        float k2vel1 = F1(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);
        float k2theta2 = vel2mid;
        float k2vel2 = F2(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);

        theta1mid = theta1 + k2theta1 * deltaTime / 2;
        vel1mid = vel1 + k2vel1 * deltaTime / 2;
        theta2mid = theta2 + k2theta2 * deltaTime / 2;
        vel2mid = vel2 + k2vel2 * deltaTime / 2;

        float k3theta1 = vel1mid;
        float k3vel1 = F1(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);
        float k3theta2 = vel2mid;
        float k3vel2 = F2(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);

        theta1mid = theta1 + k3theta1 * deltaTime;
        vel1mid = vel1 + k3vel1 * deltaTime;
        theta2mid = theta2 + k3theta2 * deltaTime;
        vel2mid = vel2 + k3vel2 * deltaTime;

        float k4theta1 = vel1mid;
        float k4vel1 = F1(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);
        float k4theta2 = vel2mid;
        float k4vel2 = F2(theta1mid, theta2mid, vel1mid, vel2mid, other_rod);

        float theta1_updated_state = theta1 + (deltaTime / 6) * (k1theta1 + 2 * k2theta1 + 2 * k3theta1 + k4theta1);
        float vel1_updated_state = vel1 + (deltaTime / 6) * (k1vel1 + 2 * k2vel1 + 2 * k3vel1 + k4vel1);
        float theta2_updated_state = theta2 + (deltaTime / 6) * (k1theta2 + 2 * k2theta2 + 2 * k3theta2 + k4theta2);
        float vel2_updated_state = vel2 + (deltaTime / 6) * (k1vel2 + 2 * k2vel2 + 2 * k3vel2 + k4vel2);
        return [theta1_updated_state, vel1_updated_state, theta2_updated_state, vel2_updated_state];
    }

    private float F1(float theta1, float theta2, float vel1, float vel2, Rod other_rod)
    {
        // A function for the first equation of motion for the double pendulum
        float g = 9.8f;
        float m1 = rod_mass;
        float m2 = other_rod.rod_mass;
        float l1 = rod_length;
        float l2 = other_rod.rod_length;

        return (-g * (2 * m1 + m2) * Sin(theta1) - m2 * g * Sin(theta1 - 2 * theta2) - 2 * Sin(theta1 - theta2) * m2 * (vel2 * vel2 * l2 + vel1 * vel1 * l1 * Cos(theta1 - theta2))) / (l1 * (2 * m1 + m2 - m2 * Cos(2 * theta1 - 2 * theta2)));
    }

    private float F2(float theta1, float theta2, float vel1, float vel2, Rod other_rod)
    {
        // A function for the second equation of motion for the double pendulum
        float g = 9.8f;
        float m1 = rod_mass;
        float m2 = other_rod.rod_mass;
        float l1 = rod_length;
        float l2 = other_rod.rod_length;

        return 2 * Sin(theta1 - theta2) * (vel1 * vel1 * l1 * (m1 + m2) + g * (m1 + m2) * Cos(theta1) + vel2 * vel2 * l2 * m2 * Cos(theta1 - theta2)) / (l2 * (2 * m1 + m2 - m2 * Cos(2 * theta1 - 2 * theta2)));
    }

    public void UpdateRotation(float deltaTime, Rod other_rod)
    {

        // Nameing variables for easier readablility 
        float theta1 = rotation;
        float theta2 = other_rod.rotation;
        float vel1 = angular_velocity;
        float vel2 = other_rod.angular_velocity;

        // Calls a function that uses the Runge-Kutta method to calcuate the pyhsics
        List<float> updated_values = CalculateRungeKutta(theta1, theta2, vel1, vel2, deltaTime, other_rod);

        // Updated the rotation based on the calculated values
        rotation = updated_values[0];
        angular_velocity = updated_values[1];
        other_rod.rotation = updated_values[2];
        other_rod.angular_velocity = updated_values[3];

    }


};

class Program // A class that holds our code
{
    static void Main()//string[] args) // Program entry point
    {
        float top_rod_length = 5f;
        float top_rod_mass = 1f;
        float bottom_rod_length = 5f;
        float bottom_rod_mass = 0.5f;
        float angle_diff = 0.1f;
        float vel_diff = 0.5f;
        bool change_vel_or_angle = true; // true for change angle, false for change velocity
        int multiplier_angle = change_vel_or_angle ? 1 : 0;
        int multiplier_vel = !change_vel_or_angle ? 1 : 0;
        string vel_or_angle = change_vel_or_angle ? "angle" : "velocity";
        int simulation_length = 100; // Length of each simulation
        float time_step_length = 0.1f;

        Rod top_rod = new Rod();
        Rod bottom_rod = new Rod();
        var path_list = new List<List<Vector2>>();
        Console.Write($"Generating Data changing the initial {vel_or_angle}");
        for (int x = 0;
            x * angle_diff * multiplier_angle + x * vel_diff * multiplier_vel < Math.PI * 2;
            x++)
        {
            float top_rod_initial_rotation = x * angle_diff * multiplier_angle;
            float top_rod_initial_velocity = x * vel_diff * multiplier_vel;
            for (int y = 0;
                y * angle_diff * multiplier_angle + y * vel_diff * multiplier_vel < Math.PI * 2;
                y++)
            {
                List<Vector2> end_path = new List<Vector2>();
                end_path.Add(new Vector2(x * angle_diff * multiplier_angle + x * vel_diff * multiplier_vel,
                                         y * angle_diff * multiplier_angle + y * vel_diff * multiplier_vel));

                float bottom_rod_initial_rotation = y * angle_diff * multiplier_angle;
                float bottom_rod_initial_velocity = y * vel_diff * multiplier_vel;
                top_rod.GiveValues(top_rod_initial_rotation,  // Initial rotation
                                top_rod_initial_velocity,  // Initial velocity
                                top_rod_length,            // Rod length
                                top_rod_mass,            // Rod mass 
                                new Vector2(0, 0)); // Rod origin

                // Finds the end of the top rod to attach the bottom one
                Vector2 top_rod_end = top_rod.FindRodEnd();

                bottom_rod.GiveValues(bottom_rod_initial_rotation, // Initial rotation
                                bottom_rod_initial_velocity,        // Initial velocity
                                bottom_rod_length,           // Rod length
                                bottom_rod_mass,        // Rod mass
                                top_rod_end); // Rod origin

                for (int time_steps = 0; time_steps < simulation_length; time_steps++)
                {
                    // Updateds the rotations and velocities of both of the rods
                    top_rod.UpdateRotation(time_step_length, bottom_rod);

                    // Centers the first rod on the screen
                    top_rod.origin_pos = new Vector2(0, 0);

                    // Attaches the second rod to the first
                    Vector2 _top_rod_end = top_rod.FindRodEnd();
                    bottom_rod.origin_pos = _top_rod_end;

                    // Records the path of the end of the second rod
                    end_path.Add(bottom_rod.FindRodEnd());
                }
                path_list.Add(end_path);

            }

        }
        foreach (List<Vector2> list in path_list)
        {
            foreach (Vector2 val in list)
            {
                Console.Write(val);
            }
        }
        var options = new JsonSerializerOptions
            {
                WriteIndented = true,
                IncludeFields = true // This makes it serialize public fields like X and Y
            };
        string jsonstring = JsonSerializer.Serialize(path_list, options);
        File.WriteAllText("C:\\Users\\nszbr\\Documents\\VS Code\\C#\\Double Pendulum\\Data Generating\\paths.json", jsonstring);
        Console.Write("Written to paths.json");
    }
}
