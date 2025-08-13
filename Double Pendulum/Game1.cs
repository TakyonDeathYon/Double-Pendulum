using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using MonoGameLibrary;

namespace Double_Pendulum;

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
    public Vector2 FindRodEnd(float img_height)
    {
        float rod_pixel_length = 0.03f * img_height * rod_length;
        float rod_end_X = origin_pos.X - rod_pixel_length * (float)Math.Sin(rotation);
        float rod_end_Y = origin_pos.Y + rod_pixel_length * (float)Math.Cos(rotation);
        return new Vector2(rod_end_X, rod_end_Y);
    }
    private float Sin(float value) {
            return (float)Math.Sin(value);
        }
    private float Cos(float value) {
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

    private float F1(float theta1, float theta2, float vel1, float vel2, Rod other_rod) {
        // A function for the first equation of motion for the double pendulum
        float g = 9.8f;
        float m1 = rod_mass;
        float m2 = other_rod.rod_mass;
        float l1 = rod_length;
        float l2 = other_rod.rod_length;

        return (-g * (2 * m1 + m2) * Sin(theta1) - m2 * g * Sin(theta1 - 2 * theta2) - 2 * Sin(theta1 - theta2) * m2 * (vel2 * vel2 * l2 + vel1 * vel1 * l1 * Cos(theta1 - theta2))) / (l1 * (2 * m1 + m2 - m2 * Cos(2 * theta1 - 2 * theta2)));
    }

    private float F2(float theta1, float theta2, float vel1, float vel2, Rod other_rod) {
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


public class Game1 : Core
{
    // The MonoGame logo texture
    private Texture2D _logo;
    private float _top_rod_length = 5f;
    private float _top_rod_mass = 1f;
    private float _top_rod_initial_rotation = 2.4f;
    private float _top_rod_initial_velocity = 0.5f;

    private float _bottom_rod_length = 5f;
    private float _bottom_rod_mass = 0.5f;
    private float _bottom_rod_initial_rotation = -2.7f;
    private float _bottom_rod_initial_velocity = 0.5f;
    private Color backgroundColour = new Color(70, 179, 202);

    private Rod _top_rod = new Rod();
    private Rod _bottom_rod = new Rod();
    private List<Vector2> _end_path = new List<Vector2>();
    KeyboardState currentKeyboardState;
    KeyboardState previousKeyboardState;


    public Game1() : base("Double Pendulum", 1280, 720, false)
    {

    }

    protected override void Initialize()
    {
        // TODO: Add your initialization logic here
        this.Window.AllowUserResizing = true;

        base.Initialize();
    }

    protected override void LoadContent()
    {
        // TODO: use this.Content to load your game content here

        _logo = Content.Load<Texture2D>("images/blacksquare");

        // Make the rod items
        _top_rod.GiveValues(_top_rod_initial_rotation,  // Initial rotation
                            _top_rod_initial_velocity,  // Initial velocity
                            _top_rod_length,            // Rod length
                            _top_rod_mass,            // Rod mass 
                            new Vector2(Window.ClientBounds.Width * 0.5f,
                            Window.ClientBounds.Height * 0.5f)); // Rod origin

        // Finds the end of the top rod to attach the bottom one
        Vector2 _top_rod_end = _top_rod.FindRodEnd(_logo.Height); 

        _bottom_rod.GiveValues(_bottom_rod_initial_rotation, // Initial rotation
                               _bottom_rod_initial_velocity,        // Initial velocity
                               _bottom_rod_length,           // Rod length
                               _bottom_rod_mass,        // Rod mass
                               _top_rod_end); // Rod origin


    }

    protected override void Update(GameTime gameTime)
    {
        if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
            Exit();

        // Updateds the rotations and velocities of both of the rods
        _top_rod.UpdateRotation((float)gameTime.ElapsedGameTime.TotalSeconds, _bottom_rod);

        // Centers the first rod on the screen
        _top_rod.origin_pos = new Vector2(Window.ClientBounds.Width * 0.5f,
                                          Window.ClientBounds.Height * 0.5f);

        // Attaches the second rod to the first
        Vector2 _top_rod_end = _top_rod.FindRodEnd(_logo.Height);
        _bottom_rod.origin_pos = _top_rod_end;

        // Records the path of the end of the second rod
        _end_path.Add(_bottom_rod.FindRodEnd(_logo.Height) - new Vector2(Window.ClientBounds.Width * 0.5f,
                                          Window.ClientBounds.Height * 0.5f));

        // Gets the state of the keyboard to listen to key presses
        currentKeyboardState = Keyboard.GetState();

        // Clear the previously recorded path 
        if (currentKeyboardState.IsKeyDown(Keys.C))
        {
            _end_path = new List<Vector2>();
        }

        // Adds velocity to the pendulum
        if (currentKeyboardState.IsKeyDown(Keys.D))
        {
            _bottom_rod.angular_velocity = _bottom_rod.angular_velocity + 0.1f * _bottom_rod.angular_velocity;
        }
        if (IsKeyPressed(Keys.E))
        {
            _top_rod.angular_velocity = _top_rod.angular_velocity + 0.1f * _top_rod.angular_velocity;
        }

        // Removes velocity from the pendulum
        if (currentKeyboardState.IsKeyDown(Keys.A))
        {
            _bottom_rod.angular_velocity = _bottom_rod.angular_velocity - 0.1f * _bottom_rod.angular_velocity;
        }
        if (currentKeyboardState.IsKeyDown(Keys.Q))
        {
            _top_rod.angular_velocity = _top_rod.angular_velocity - 0.1f * _top_rod.angular_velocity;
           
        }



        // Allows reactions to only initial presses, not every frame the key is down
        previousKeyboardState = currentKeyboardState;


        base.Update(gameTime);
    }
    private bool IsKeyPressed(Keys key)
    {
    return currentKeyboardState.IsKeyDown(key) && previousKeyboardState.IsKeyUp(key);
    }   
    

    protected override void Draw(GameTime gameTime)
    {
        GraphicsDevice.Clear(backgroundColour);

        // Begin the sprite batch to prepare for rendering.
        SpriteBatch.Begin();

        // Draw _top_rod item
        SpriteBatch.Draw(_logo,
            _top_rod.origin_pos,
            null,               // source rectangle 
            Color.White * 0.5f,        // tint
            _top_rod.rotation,               // rotation
            new Vector2(
                _logo.Width,
                0f) * 0.5f,       // origin
            new Vector2(
                0.03f,
                0.03f * _top_rod.rod_length
            ),               // scale
            SpriteEffects.None, // effects
            0.0f                // layer depth
            );

        // Draw _bottom_rod item
        SpriteBatch.Draw(_logo,
            _bottom_rod.origin_pos,
            null,               // source rectangle 
            Color.White * 0.5f,        // tint
            _bottom_rod.rotation,               // rotation
            new Vector2(
                _logo.Width,
                0f) * 0.5f,       // origin
            new Vector2(
                0.03f,
                0.03f * _bottom_rod.rod_length
            ),               // scale
            SpriteEffects.None, // effects
            0.0f                // layer depth
            );
        for (int i = 0; i < _end_path.Count; i++)
        {
            SpriteBatch.Draw(_logo,
            _end_path[i] + new Vector2(Window.ClientBounds.Width * 0.5f,
                                          Window.ClientBounds.Height * 0.5f),
            null,               // source rectangle 
            Color.Red * 0.5f,        // tint
            0.0f,               // rotation
            new Vector2(
                _logo.Width,
                0f) * 0.5f,       // origin
            new Vector2(
                0.003f,
                0.003f
            ),               // scale
            SpriteEffects.None, // effects
            0.0f                // layer depth
            );
        }
        // Always end the sprite batch when finished.
        SpriteBatch.End();

        // TODO: Add your drawing code here

        base.Draw(gameTime);
    }
}
