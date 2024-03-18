namespace pid
{
    class PID
    {
        public:
            PID(float Kp, float Ki, float Kd, float limMax, float limMin, float dt);
            float update(float setpoint, float mesurement);

        private:
            float Kp;
            float Ki;
            float Kd;

            float limMax;
            float limMin;

            float dt;

            float integral;
            float prevError;
            float derivative;
            float prevMessurement;

            float output;
    };
}