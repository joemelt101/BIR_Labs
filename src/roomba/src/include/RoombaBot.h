#pragma once

namespace irobot
{
    class TurtleController
    {
        public:
            TurtleController();
            void update();
            void setVelocity(float velocity);
            void setAngVelocity(float angularVelocity);
            void moveFoward(float distance);
        private:
            float _velocity;
            float _angularVelocity;
            float _originX;
            float _originY;
    };
}