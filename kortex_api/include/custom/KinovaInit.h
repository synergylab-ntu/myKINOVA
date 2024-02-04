#include <string>

#include <BaseClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

class KINOVA
{
public:
	KINOVA(const std::string& ip_address, int port, const std::string& username = "admin", const std::string& password = "admin"):
        m_ip_address(ip_address), m_username(username), m_password(password), m_port(port)
	{
        m_router = nullptr;
        m_transport_tcp = nullptr;
        m_session_manager = nullptr;
        m_device_manager = nullptr;
        m_base = nullptr;

        m_is_init = false;
	}
		
	~KINOVA() 
	{
        // Close API session
        m_session_manager->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        m_router->SetActivationStatus(false);
        m_transport_tcp->disconnect();

        // Destroy the API
        delete m_base;
        delete m_device_manager;
        delete m_session_manager;
        delete m_router;
        delete m_transport_tcp;
	}

    void Init_TCP()
    {
        if (m_is_init)
        {
            return;
        }
        m_transport_tcp = new k_api::TransportClientTcp();
        if (m_port == 10000)
        {
            m_transport_tcp->connect(m_ip_address, m_port);
        }
        else
        {
            std::cout << "Wrong port for TCP" << std::endl;
            return;
        }
        m_router = new k_api::RouterClient(m_transport_tcp, [](k_api::KError err) { std::cout << "_________ callback error _________" << err.toString(); });

        // Set session data connection information
        auto createSessionInfo = k_api::Session::CreateSessionInfo();
        createSessionInfo.set_username(m_username);
        createSessionInfo.set_password(m_password);
        createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
        createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

        // Session manager service wrapper
        std::cout << "Creating TCP session for communication" << std::endl;
        m_session_manager = new k_api::SessionManager(m_router);
        m_session_manager->CreateSession(createSessionInfo);
        std::cout << "TCP Session created" << std::endl;

        m_is_init = true;
    }

    void Init_UDP()
    {
        if (m_is_init)
        {
            return;
        }
        m_transport_udp = new k_api::TransportClientUdp();
        if (m_port == 10001)
        {
            m_transport_udp->connect(m_ip_address, m_port);
        }
        else
        {
            std::cout << "Wrong port for UDP" << std::endl;
            return;
        }
        m_router = new k_api::RouterClient(m_transport_udp, [](k_api::KError err) { std::cout << "_________ callback error _________" << err.toString(); });

        // Set session data connection information
        auto createSessionInfo = k_api::Session::CreateSessionInfo();
        createSessionInfo.set_username(m_username);
        createSessionInfo.set_password(m_password);
        createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
        createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

        // Session manager service wrapper
        std::cout << "Creating UDP session for communication" << std::endl;
        m_session_manager = new k_api::SessionManager(m_router);
        m_session_manager->CreateSession(createSessionInfo);
        std::cout << "UDP Session created" << std::endl;

        m_is_init = true;
    }

    k_api::RouterClient* get_router()
    {
        return m_router;
    }

private:
    k_api::RouterClient* m_router;
    k_api::TransportClientTcp* m_transport_tcp;
    k_api::TransportClientUdp* m_transport_udp;
    k_api::SessionManager* m_session_manager;
    k_api::Base::BaseClient* m_base;
    k_api::DeviceManager::DeviceManagerClient* m_device_manager;
    bool                                         m_is_init;
    std::string                                  m_username;
    std::string                                  m_password;
    std::string                                  m_ip_address;
    int                                          m_port;
};