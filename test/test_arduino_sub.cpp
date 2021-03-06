#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

// TODO: add other includes as needed
#include <arduino_msgs/states.h>
#include <arduino_msgs/commands.h>

#define __TEST_NMEA__
#define __TEST_COMMANDS__

#define __TEST_NMEA_REPLACE_ENDINGS__

#ifdef __TEST_NMEA__
// libnmea (external project)
#include <nmea.h>
#include <nmea/gprmc.h>

#include <regex>
#endif

namespace ros_arduino {
class ROSArduinoTest : public ::testing::Test {
public:
    ROSArduinoTest()
        : node_handle_(),
          publisher_(
              node_handle_.advertise<arduino_msgs::commands>(
                  "/Arduino/sub/cmds", 5)),
          subscriber_(
              node_handle_.subscribe("/Arduino/pub/states", 5,
                                     &ROSArduinoTest::Callback,
                                     this)),
          message_received_(false) {
    }

    /*
   * This is necessary because it takes a while for the node under
   * test to start up.
   */
    void SetUp() {
        while (!IsNodeReady()) {
            ros::spinOnce();
        }
    }

    arduino_msgs::commands Publish(
            bool update_clock
            , uint8_t t2
            , uint8_t t3
            , uint8_t t4
            , bool state_start
            , bool state_pause
            , bool state_flash
            , bool state_boot
            , bool state_halt
            ) {
        arduino_msgs::commands message;

        // TODO: construct message.
        message.update_clock = update_clock;
        message.t2_t3_t4[0] = t2;
        message.t2_t3_t4[1] = t3;
        message.t2_t3_t4[2] = t4;
        message.state_boot = state_boot;
        message.state_flash = state_flash;
        message.state_halt = state_halt;
        message.state_start = state_start;
        message.state_pause = state_pause;

        publisher_.publish(message);
        return message;
    }

    void Publish(arduino_msgs::commands message) {
        publisher_.publish(message);
    }

    /*
   * This is necessary because it takes time for messages from the
   * node under test to reach this node.
   */
    boost::shared_ptr<const arduino_msgs::states> WaitForMessage() {
        // The second parameter is a timeout duration.
        return ros::topic::waitForMessage<arduino_msgs::states>(
                    subscriber_.getTopic(), ros::Duration(1));
    }

    // An alternative way of waiting for a message.
    // ros::topic::waitForMessage can sometimes be flaky.
    void WaitForMessageAlternate() {
        while(!message_received_) {
            ros::spinOnce();
        }
    }

    /**
     * @brief WaitForMessageAlternate_ArduinoStates
     */
    void WaitForMessageAlternate_ArduinoStates() {
        WaitForMessageAlternate();

        gprmc_conditionned_ = states_.gprmc;

#ifdef __TEST_NMEA_REPLACE_ENDINGS__
        // urls:
        // - http://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
        // - https://fr.wikipedia.org/wiki/NMEA_0183
        // On retire les caractères de fin de chaine '\r' dans notre génération de message
        // et on les remplace par les caractères de terminaisons reconnus par la libnmea qui sont
        // "\n\n".
        // TODO: Essayer de remplacer directement les caractères de terminaison de la chaine NMEA sentence
        // produit par l'Arduino. Faudrait vérifier cette notion de standard sur ces fins de messages ...
        gprmc_conditionned_.replace(gprmc_conditionned_.find('\r'), sizeof("\n\n")-1, "\n\n");
#endif

        MessageConsummed();
    }

    //
    inline void MessageConsummed() {
        message_received_ = false;
    }

    inline const arduino_msgs::states& GetStates() const {
        return states_;
    }

    inline const char* GetSentence() const {
        return gprmc_conditionned_.c_str();
    }

private:
    ros::NodeHandle node_handle_;

    ros::Publisher  publisher_;
    ros::Subscriber subscriber_;

    bool message_received_;

    arduino_msgs::states states_;
    arduino_msgs::states::_gprmc_type gprmc_conditionned_;

    /*
   * This callback is a no-op because we get the messages from the
   * node under test using WaitForMessage().
   */
    void Callback(const arduino_msgs::states& event) {
        message_received_ = true;
        // TODO: sauvegarder l'event pour l'exploiter par la suite dans les tests!
        states_ = event;
    }

    /*
   * See SetUp method.   */
    bool IsNodeReady() {
        return (publisher_.getNumSubscribers() > 0)
                && (subscriber_.getNumPublishers() > 0);
        //        return (subscriber_.getNumPublishers() > 0);
    }
};

TEST_F(ROSArduinoTest, TestSubscribeToArduinoStatesFromROSSerial) {
    WaitForMessageAlternate();
    MessageConsummed();

    ASSERT_TRUE(true);
}

//#define DEBUG_SENTENCE_OUTPUT
#ifdef __TEST_NMEA__
TEST_F(ROSArduinoTest, TestValidateNMEA_TypeSentence) {
    WaitForMessageAlternate_ArduinoStates();
    char* sentence = const_cast<char*>(GetSentence());
#ifdef DEBUG_SENTENCE_OUTPUT
    std::cerr << "sentence: " << sentence << std::endl;
#endif

    ASSERT_EQ(nmea_get_type(sentence), NMEA_GPRMC) << "type sentence is not NMEA_GPRMC"
                                                   << "- sentence: " << sentence;
}

TEST_F(ROSArduinoTest, TestValidateNMEA_CheckSum) {
    WaitForMessageAlternate_ArduinoStates();
    char* sentence = const_cast<char*>(GetSentence());
#ifdef DEBUG_SENTENCE_OUTPUT
    std::cerr << "sentence: " << sentence << std::endl;
#endif

    ASSERT_EQ(nmea_has_checksum(sentence, strlen(sentence)), 0) << "sentence has no (valid) checksum"
                                                                << "- sentence: " << sentence;
}

TEST_F(ROSArduinoTest, TestValidateNMEA_Validate) {
    WaitForMessageAlternate_ArduinoStates();
    char* sentence = const_cast<char*>(GetSentence());
    const size_t strlen_sentence = strlen(sentence);
#ifdef DEBUG_SENTENCE_OUTPUT
    std::cerr << "sentence: " << sentence << std::endl;
#endif

    ASSERT_EQ(nmea_validate(sentence,
                            strlen_sentence,
                            nmea_has_checksum(sentence, strlen_sentence)==0),
              0) << "sentence is not valid"
                 << "- sentence: " << sentence;
}

TEST_F(ROSArduinoTest, TestValidateNMEA_Parse) {
    WaitForMessageAlternate_ArduinoStates();
    char* sentence = const_cast<char*>(GetSentence());
    const size_t strlen_sentence = strlen(sentence);
#ifdef DEBUG_SENTENCE_OUTPUT
    std::cerr << "sentence: " << sentence << std::endl;
#endif    

    nmea_s *data = nmea_parse(sentence,
                              strlen_sentence,
                              nmea_has_checksum(sentence, strlen_sentence) == 0);

    ASSERT_TRUE(data!=NULL) << "Can't parse the sentence"
                            << "- sentence: " << sentence;

    if(data) {
        ASSERT_EQ(data->errors, 0) << "The sentence struct contains "
                                   << data->errors << " parse errors!"
                                   << "- sentence: " << sentence;
        nmea_free(data);
    }
}

TEST_F(ROSArduinoTest, TestValidateNMEA_Parse_Time) {
    WaitForMessageAlternate_ArduinoStates();
    char* sentence = const_cast<char*>(GetSentence());
    const size_t strlen_sentence = strlen(sentence);
#ifdef DEBUG_SENTENCE_OUTPUT
    std::cerr << "sentence: " << sentence << std::endl;
#endif
    nmea_s *data = nmea_parse(sentence,
                              strlen_sentence,
                              nmea_has_checksum(sentence, strlen_sentence)==0);

    if(data && (data->errors==0)) {
        nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
        const arduino_msgs::states& states = GetStates();
        // url: http://www.cplusplus.com/reference/ctime/tm/
        EXPECT_EQ(pos->time.tm_sec, states.t2_t3_t4[0]) <<  "Time: second not synchronize !"
                                                         << "- sentence: " << sentence;
        EXPECT_EQ(pos->time.tm_min, states.t2_t3_t4[1]) <<  "Time: minute not synchronize !"
                                                         << "- sentence: " << sentence;
        EXPECT_EQ(pos->time.tm_hour, states.t2_t3_t4[2]) << "Time: hour not synchronize !"
                                                         << "- sentence: " << sentence;
        //
        nmea_free(data);
    }
}
#endif  // __TEST_NMEA__

#ifdef __TEST_COMMANDS__
TEST_F(ROSArduinoTest, TestValidatePublisherCommands_ALL) {
    arduino_msgs::commands commands;
    //
    commands.update_clock = true;
    commands.t2_t3_t4[0] = rand() % 60;
    commands.t2_t3_t4[1] = rand() % 60;
    commands.t2_t3_t4[2] = rand() % 24;
    commands.state_flash = bool(rand() % 2);
    commands.state_start = bool(rand() % 2);
    commands.state_pause = bool(rand() % 2);
    commands.state_boot = bool(rand() % 2);
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ASSERT_EQ(states.state_boot, commands.state_boot);
    ASSERT_EQ(states.state_pause, commands.state_pause);
    ASSERT_EQ(states.state_start, commands.state_start);
    ASSERT_EQ(states.state_flash, commands.state_flash);
    ++commands.t2_t3_t4[0]; // car double appel de WaitForMessageAlternate_ArduinoStates()
    ASSERT_EQ(states.t2_t3_t4, commands.t2_t3_t4);
}

TEST_F(ROSArduinoTest, TestValidatePublisherCommands_Clock) {
    arduino_msgs::commands commands;
    //
    commands.update_clock = true;
    commands.t2_t3_t4[0] = rand() % 60;
    commands.t2_t3_t4[1] = rand() % 60;
    commands.t2_t3_t4[2] = rand() % 24;
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ++commands.t2_t3_t4[0]; // car double appel de WaitForMessageAlternate_ArduinoStates()
    ASSERT_EQ(states.t2_t3_t4, commands.t2_t3_t4);
}

TEST_F(ROSArduinoTest, TestValidatePublisherCommands_Flash) {
    arduino_msgs::commands commands;
    commands.state_flash = bool(rand() % 2);
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ASSERT_EQ(states.state_flash, commands.state_flash);
}

TEST_F(ROSArduinoTest, TestValidatePublisherCommands_Start) {
    arduino_msgs::commands commands;
    commands.state_start = bool(rand() % 2);
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ASSERT_EQ(states.state_start, commands.state_start);
}

TEST_F(ROSArduinoTest, TestValidatePublisherCommands_Pause) {
    arduino_msgs::commands commands;
    commands.state_pause = bool(rand() % 2);
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ASSERT_EQ(states.state_pause, commands.state_pause);
}

TEST_F(ROSArduinoTest, TestValidatePublisherCommands_Boot) {
    arduino_msgs::commands commands;
    commands.state_boot = bool(rand() % 2);
    //
    Publish(commands);
    //
    WaitForMessageAlternate_ArduinoStates();
    WaitForMessageAlternate_ArduinoStates();
    //
    const arduino_msgs::states& states = GetStates();
    //
    ASSERT_EQ(states.state_boot, commands.state_boot);
}
#endif  // __TEST_COMMANDS__

}   // namespace ros_arduino

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_arduino_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
