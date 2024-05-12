// Funkenstein language

#include <actions/execute_script.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

class Line
{
public:
    virtual void run();
};

class SetAnglesLine : Line
{
public:
    void run()
    {
    }
};

struct State
{
    ros::NodeHandle nh;
    ros::ServiceServer service;

    State()
    {
        nh.advertiseService("funk", &State::handle_service, this);
    }

    bool handle_service(actions::execute_scriptRequest &req, actions::execute_scriptResponse &res)
    {
        const std::string &script_name = req.script_name;
        std::string script = read_to_file(script_name);
        std::vector<std::string> script_lines = lines(script, '\n');

        return true;
    }

    static std::string read_to_file(const std::string &filename)
    {
        FILE *script_file = fopen(filename.c_str(), "r");
        assert(script_file);
        fseek(script_file, 0, SEEK_END);
        size_t script_len = ftell(script_file);

        std::string script;
        script.resize(script_len);
        fread((void *)script.data(), 1, script_len, script_file);
        fclose(script_file);

        return script;
    }

    static std::vector<std::string> lines(const std::string &str, char by)
    {
        int line_start = 0;
        int line_end = 0;

        std::vector<std::string> result;
        while (line_end < str.size())
        {
            if (str[line_end] == by)
            {
                result.push_back(str.substr(line_start, line_end));
                line_start = line_end + 1;
            }

            line_end++;
        }

        if (line_start != line_end)
            result.push_back(str.substr(line_start, line_end));

        return result;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "funkenstein");
    State state;
    ros::spin();
}
