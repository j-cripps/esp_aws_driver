set(COMPONENT_SRCS "example_driver.c")

register_component()

if(CONFIG_EXAMPLE_EMBEDDED_CERTS)
target_add_binary_data(${COMPONENT_TARGET} "../certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "../certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "../certs/private.pem.key" TEXT)
endif()
