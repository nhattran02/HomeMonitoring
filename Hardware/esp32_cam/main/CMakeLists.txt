set(srcs "main.c"
	     "connect_wifi.c")
idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
EMBED_TXTFILES telegram_certificate.pem)

                    # Embed the server root certificate into the final binary
