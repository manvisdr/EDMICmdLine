#include <config.h>

#include <cstddef>
#include <cstdint>
#include <map>

#include <HardwareSerial.h>
#include <pgmspace.h>

typedef uint8_t byte;

class EdmiCMDReader
{
public:
    enum class Status : uint8_t
    {
        Connect,
        Disconnect,
        Ready,
        LoggedIn,
        NotLogin,
        Busy,
        Finish,
        TimeoutError,
        ProtocolError,
        ChecksumError
    };

    const std::map<Status, std::string> EDMI_STATUS_MAP = {
        {Status::Disconnect, "Disconnect"},
        {Status::Connect, "Connect"},
        {Status::Ready, "Ready"},
        {Status::LoggedIn, "LoggedIn"},
        {Status::NotLogin, "NotLogin"},
        {Status::Busy, "Busy"},
        {Status::Finish, "Finish"},
        {Status::TimeoutError, "Err-Timeout"},
        {Status::ProtocolError, "Err-Prot"},
        {Status::ChecksumError, "Err-Chk"}};

    explicit EdmiCMDReader(HardwareSerial &serial, uint8_t rxpin, uint8_t txpin) : serial_(serial)
    {
        rx_ = rxpin;
        tx_ = txpin;
    }

    EdmiCMDReader(EdmiCMDReader const &) = delete;

    EdmiCMDReader(EdmiCMDReader &&) = delete;

    void begin(unsigned long baud);

    void keepAlive();
    void read_looping();

    void TX_raw(uint8_t d);
    void TX_cmd(uint8_t *cmd, unsigned short len);

    void send_cmdR(const byte *reg);

    void step_start();
    bool read_default();

    String serialNumber();
    float voltR();
    float voltS();
    float voltT();
    float currentR();
    float currentS();
    float currentT();
    float wattR();
    float wattS();
    float wattT();
    float pf();
    float frequency();
    float kVarh();
    float kwhWBP();
    float kwhLWBP();
    float kwhTotal();

    float read_voltR();
    float read_voltS();
    float read_voltT();
    float read_currentR();
    float read_currentS();
    float read_currentT();
    float read_wattR();
    float read_wattS();
    float read_wattT();
    float read_pf();
    float read_frequency();
    float read_kVarh();
    float read_kwhWBP();
    float read_kwhLWBP();
    float read_kwhTotal();

    String read_Serialnumber(/*char *output, int len*/);

    Status status() const { return status_; }
    std::string printStatus()
    {
        status_ = status();
        auto it = EDMI_STATUS_MAP.find(status_);
        if (it == EDMI_STATUS_MAP.end())
            return "Unknw";
        return it->second;
    }

    void acknowledge()
    {
        Serial.println("ACKNOWLEDGE");
        if (status_ != Status::Busy and status_ != Status::Disconnect)
            status_ = Status::Ready;
    }

protected:
    enum class Step : uint8_t;
    enum class ErrorCode : uint8_t
    {
        NullError,
        CannotWrite,
        UnimplementedOperation,
        RegisterNotFound,
        AccessDenied,
        WrongLength,
        BadTypCode,
        DataNotReady,
        OutOfRange,
        NotLoggedIn,
    };

    struct
    {
        String serialNumber;
        float voltR;
        float voltS;
        float voltT;
        float currentR;
        float currentS;
        float currentT;
        float wattR;
        float wattS;
        float wattT;
        float pf;
        float frequency;
        float kVarh;
        float kwhWBP;
        float kwhLWBP;
        float kwhTotal;
    } _currentValues; // Measured values

    const std::map<ErrorCode, std::string> EDMI_ERROR_MAP = {
        {ErrorCode::NullError, "Null Error"},
        {ErrorCode::CannotWrite, "Cannot Write"},
        {ErrorCode::UnimplementedOperation, "Unimplemented Operation"},
        {ErrorCode::RegisterNotFound, "Register Not Found"},
        {ErrorCode::AccessDenied, "Access Denied"},
        {ErrorCode::WrongLength, "Wrong Length"},
        {ErrorCode::BadTypCode, "Bad Type Code"},
        {ErrorCode::DataNotReady, "Data Not Ready"},
        {ErrorCode::OutOfRange, "Out Of Range"},
        {ErrorCode::NotLoggedIn, "Not Logged In"}};

    ErrorCode errorCode() const { return regError_; }

    std::string printError()
    {
        regError_ = errorCode();
        auto it = EDMI_ERROR_MAP.find(regError_);
        if (it == EDMI_ERROR_MAP.end())
            return "Unknw";
        return it->second;
    }

    void step_login();
    void step_logout();
    void step_read();
    void TX_byte(uint8_t d);
    bool RX_char(unsigned int timeout, byte *inChar);
    bool check_RXReg(uint8_t *buff, uint8_t typeReg, uint8_t compare);

    uint8_t RX_message(uint8_t *message, int maxMessageSize, unsigned int timeout);
    void change_status(Status to);

    Step step_;
    ErrorCode regError_;
    size_t errors_ = 0, checksum_errors_ = 0, successes_ = 0;
    HardwareSerial &serial_;
    Status status_ = Status::Ready;
    uint8_t rx_, tx_;
};