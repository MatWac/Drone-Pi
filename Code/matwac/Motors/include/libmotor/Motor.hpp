namespace motor
{
    class Motor
    {
        public:
            Motor(int pin);
            ~Motor();
            void setSpeed(int level);
	    int getSpeed() const;

        private:
            int m_pin;
            int m_level;

    };
}

