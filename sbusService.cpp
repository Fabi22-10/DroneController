#include "sbusService.h"

static void onPacket(const sbus_packet_t &packet)
{
    static auto lastPrint = steady_clock::now();
    auto now = steady_clock::now();

    if (now - lastPrint > milliseconds(500))
    {
        for (int i = 0; i < 16; ++i)
            cout << "ch" << i + 1 << ": " << packet.channels[i] << "\t";

        cout << "ch17: " << (packet.ch17 ? "true" : "false") << "\t"
             << "ch18: " << (packet.ch18 ? "true" : "false");

        if (packet.frameLost)
            cout << "\tFrame lost";

        if (packet.failsafe)
            cout << "\tFailsafe active";

        cout << endl;

        lastPrint = now;
    }
}

int sbusService() 
{


}
