#!/usr/bin/env python3
"""
Implements mutual authentication using certificates and PUF-based lightweight authentication
"""

import socket
import threading
import time
import json
import hashlib
import secrets
import struct
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography import x509
from cryptography.x509.oid import NameOID
import datetime
from typing import Dict, Tuple, Optional
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

class PUFSimulator:
    """Simulates Physical Unclonable Function (PUF) behavior"""
    
    def __init__(self, device_id: str):
        self.device_id = device_id
        # Use device ID as seed for reproducible but unique PUF behavior
        self.seed = hashlib.sha256(device_id.encode()).hexdigest()
    
    def generate_response(self, challenge: bytes) -> bytes:
        """Generate PUF response for a given challenge"""
        combined = challenge + self.seed.encode()
        return hashlib.sha256(combined).digest()[:16]  # 128-bit response

class CertificateAuthority:
    """Certificate Authority for issuing X.509 certificates with ECC P-192 (192-bit keys)"""
    
    def __init__(self):
        # Use ECC P-192 for smaller key size
        self.private_key = ec.generate_private_key(ec.SECP192R1())
        self.public_key = self.private_key.public_key()
        self.ca_cert = self._create_ca_certificate()
        
        # Log key size for verification
        key_size = len(self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
        print(f"📏 CA Public Key Size: {key_size} bytes")
        
    def _create_ca_certificate(self) -> x509.Certificate:
        """Create self-signed CA certificate"""
        subject = issuer = x509.Name([
            x509.NameAttribute(NameOID.COUNTRY_NAME, "MR"),
            x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "RAK"),
            x509.NameAttribute(NameOID.LOCALITY_NAME, "BEN GUERIR"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Drone Network CA"),
            x509.NameAttribute(NameOID.COMMON_NAME, "Drone CA Root"),
        ])
        
        cert = x509.CertificateBuilder().subject_name(
            subject
        ).issuer_name(
            issuer
        ).public_key(
            self.public_key
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            datetime.datetime.utcnow()
        ).not_valid_after(
            datetime.datetime.utcnow() + datetime.timedelta(days=365)
        ).sign(self.private_key, hashes.SHA256())
        
        return cert
    
    def issue_certificate(self, device_id: str, public_key) -> x509.Certificate:
        """Issue certificate for a device"""
        subject = x509.Name([
            x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Drone Network"),
            x509.NameAttribute(NameOID.COMMON_NAME, device_id),
        ])
        
        cert = x509.CertificateBuilder().subject_name(
            subject
        ).issuer_name(
            self.ca_cert.subject
        ).public_key(
            public_key
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            datetime.datetime.utcnow()
        ).not_valid_after(
            datetime.datetime.utcnow() + datetime.timedelta(days=30)
        ).sign(self.private_key, hashes.SHA256())
        
        return cert

class MessageProtocol:
    """Handles message formatting and parsing"""
    
    @staticmethod
    def send_message(sock: socket.socket, msg_type: str, data: dict):
        """Send a message"""
        message = {'type': msg_type, 'data': data}
        message_json = json.dumps(message).encode('utf-8')
        length = len(message_json)
        sock.send(struct.pack('!I', length) + message_json)
    
    @staticmethod
    def receive_message(sock: socket.socket) -> Tuple[str, dict]:
        """Receive a complete message"""
        # Receive length
        length_data = sock.recv(4)
        if len(length_data) != 4:
            raise ValueError("Failed to receive message length")
        
        length = struct.unpack('!I', length_data)[0]
        
        # Receive message data
        message_data = b''
        while len(message_data) < length:
            chunk = sock.recv(length - len(message_data))
            if not chunk:
                raise ValueError("Connection closed")
            message_data += chunk
        
        message = json.loads(message_data.decode('utf-8'))
        return message['type'], message['data']

class SecureChannel:
    """Handles encrypted communication"""
    
    def __init__(self, session_key: bytes):
        self.session_key = session_key
    
    def encrypt(self, data: bytes) -> bytes:
        """Encrypt data using AES-GCM"""
        iv = secrets.token_bytes(12)
        cipher = Cipher(algorithms.AES(self.session_key), modes.GCM(iv))
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(data) + encryptor.finalize()
        return iv + encryptor.tag + ciphertext
    
    def decrypt(self, encrypted_data: bytes) -> bytes:
        """Decrypt data using AES-GCM"""
        iv = encrypted_data[:12]
        tag = encrypted_data[12:28]
        ciphertext = encrypted_data[28:]
        cipher = Cipher(algorithms.AES(self.session_key), modes.GCM(iv, tag))
        decryptor = cipher.decryptor()
        return decryptor.update(ciphertext) + decryptor.finalize()
    
    def send_encrypted(self, sock: socket.socket, msg_type: str, data: dict):
        """Send encrypted message"""
        message = {'type': msg_type, 'data': data}
        message_json = json.dumps(message).encode('utf-8')
        encrypted = self.encrypt(message_json)
        sock.send(struct.pack('!I', len(encrypted)) + encrypted)
    
    def receive_encrypted(self, sock: socket.socket) -> Tuple[str, dict]:
        """Receive encrypted message"""
        length_data = sock.recv(4)
        length = struct.unpack('!I', length_data)[0]
        encrypted_data = sock.recv(length)
        decrypted = self.decrypt(encrypted_data)
        message = json.loads(decrypted.decode('utf-8'))
        return message['type'], message['data']

class Gateway:
    """Gateway/Base Station implementation"""
    
    def __init__(self, gateway_id: str, host: str = 'localhost', port: int = 8282):
        self.gateway_id = gateway_id
        self.host = host
        self.port = port
        self.logger = logging.getLogger(f"Gateway-{gateway_id}")
        
        # Generate ECC P-192 key pair (192-bit)
        self.private_key = ec.generate_private_key(ec.SECP192R1())
        self.public_key = self.private_key.public_key()
        
        # Log key size for verification
        key_size = len(self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
        self.logger.info(f"Gateway key size: {key_size} bytes")
        
        # Initialize PUF
        self.puf = PUFSimulator(gateway_id)
        
        # Storage
        self.certificate = None
        self.ca_certificate = None
        self.drone_puf_profiles = {}
        self.authenticated_drones = set()
        
        self.server_socket = None
        self.running = False
        
    def set_certificate(self, certificate: x509.Certificate, ca_cert: x509.Certificate):
        """Set certificates"""
        self.certificate = certificate
        self.ca_certificate = ca_cert
        
    def start_server(self):
        """Start the gateway server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.running = True
        
        self.logger.info(f"Gateway {self.gateway_id} listening on {self.host}:{self.port}")
        
        while self.running:
            try:
                client_socket, address = self.server_socket.accept()
                if not self.running:
                    break
                    
                self.logger.info(f"Connection from {address}")
                
                thread = threading.Thread(
                    target=self.handle_drone_connection,
                    args=(client_socket,),
                    daemon=True
                )
                thread.start()
                
            except socket.error as e:
                if self.running:
                    self.logger.error(f"Socket error: {e}")
                break
    
    def handle_drone_connection(self, client_socket: socket.socket):
        """Handle drone authentication"""
        try:
            # Receive hello message
            msg_type, msg_data = MessageProtocol.receive_message(client_socket)
            
            if msg_type == "HELLO":
                drone_id = msg_data["drone_id"]
                self.logger.info(f"Received HELLO from drone {drone_id}")
                
                if drone_id in self.authenticated_drones:
                    # Use PUF authentication for known drones
                    self._handle_puf_authentication(client_socket, drone_id)
                else:
                    # Use certificate authentication for new drones
                    self._handle_certificate_authentication(client_socket, drone_id)
                    
        except Exception as e:
            self.logger.error(f"Error handling drone connection: {e}")
        finally:
            client_socket.close()
    
    def _handle_certificate_authentication(self, client_socket: socket.socket, drone_id: str):
        """Handle certificate-based authentication"""
        try:
            self.logger.info(f"Starting certificate authentication with {drone_id}")
            
            # Send our certificate
            cert_pem = self.certificate.public_bytes(serialization.Encoding.PEM).decode()
            MessageProtocol.send_message(client_socket, "CERT_GATEWAY", {"certificate": cert_pem})
            
            # Receive drone certificate
            msg_type, msg_data = MessageProtocol.receive_message(client_socket)
            if msg_type != "CERT_DRONE":
                raise ValueError("Expected CERT_DRONE")
            
            drone_cert_pem = msg_data["certificate"].encode()
            drone_cert = x509.load_pem_x509_certificate(drone_cert_pem)
            
            # Verify drone certificate
            if not self._verify_certificate(drone_cert):
                raise ValueError("Certificate verification failed")
            
            self.logger.info(f"Certificate verified for {drone_id}")
            
            # Perform key exchange
            session_key = self._perform_key_exchange(client_socket)
            secure_channel = SecureChannel(session_key)
            
            # Enroll PUF profiles
            self._enroll_puf_profiles(client_socket, secure_channel, drone_id)
            
            # Mark as authenticated
            self.authenticated_drones.add(drone_id)
            self.logger.info(f"Certificate authentication completed for {drone_id}")
            
            # Send success
            secure_channel.send_encrypted(client_socket, "AUTH_SUCCESS", {})
            
        except Exception as e:
            self.logger.error(f"Certificate authentication failed: {e}")
    
    def _handle_puf_authentication(self, client_socket: socket.socket, drone_id: str):
        """Handle PUF-based authentication"""
        try:
            self.logger.info(f"Starting PUF authentication with {drone_id}")
            
            # Perform key exchange
            session_key = self._perform_key_exchange(client_socket)
            secure_channel = SecureChannel(session_key)
            
            # Get stored PUF profile
            if drone_id not in self.drone_puf_profiles:
                raise ValueError(f"No PUF profile for {drone_id}")
            
            challenge, expected_response = self.drone_puf_profiles[drone_id]
            
            # Send challenge to drone
            secure_channel.send_encrypted(client_socket, "PUF_CHALLENGE", {"challenge": challenge.hex()})
            
            # Receive response
            msg_type, msg_data = secure_channel.receive_encrypted(client_socket)
            if msg_type != "PUF_RESPONSE":
                raise ValueError("Expected PUF_RESPONSE")
            
            drone_response = bytes.fromhex(msg_data["response"])
            if drone_response != expected_response:
                raise ValueError("Invalid PUF response")
            
            self.logger.info(f"PUF authentication successful for {drone_id}")
            
            # Send success
            secure_channel.send_encrypted(client_socket, "AUTH_SUCCESS", {})
            
        except Exception as e:
            self.logger.error(f"PUF authentication failed: {e}")
    
    def _verify_certificate(self, certificate: x509.Certificate) -> bool:
        """Verify certificate"""
        try:
            # Check validity period
            now = datetime.datetime.utcnow()
            if now < certificate.not_valid_before or now > certificate.not_valid_after:
                self.logger.error("Certificate expired")
                return False
            
            # Verify signature
            ca_public_key = self.ca_certificate.public_key()
            ca_public_key.verify(
                certificate.signature,
                certificate.tbs_certificate_bytes,
                ec.ECDSA(certificate.signature_hash_algorithm)
            )
            
            self.logger.info("Certificate verification passed")
            return True
            
        except Exception as e:
            self.logger.error(f"Certificate verification failed: {e}")
            return False
    
    def _perform_key_exchange(self, client_socket: socket.socket) -> bytes:
        """Perform ECDHE key exchange"""
        # Generate ephemeral key pair (P-192 for smaller keys)
        private_key = ec.generate_private_key(ec.SECP192R1())
        public_key = private_key.public_key()
        
        # Send our public key
        public_key_pem = public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ).decode()
        
        MessageProtocol.send_message(client_socket, "ECDHE_PUBLIC", {"public_key": public_key_pem})
        
        # Receive peer's public key
        msg_type, msg_data = MessageProtocol.receive_message(client_socket)
        if msg_type != "ECDHE_PUBLIC":
            raise ValueError("Expected ECDHE_PUBLIC")
        
        peer_public_key = serialization.load_pem_public_key(msg_data["public_key"].encode())
        
        # Compute shared secret
        shared_key = private_key.exchange(ec.ECDH(), peer_public_key)
        
        # Derive session key
        session_key = HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=b'session_key',
        ).derive(shared_key)
        
        return session_key
    
    def _enroll_puf_profiles(self, client_socket: socket.socket, secure_channel: SecureChannel, drone_id: str):
        """Enroll PUF profiles"""
        # Generate challenge for drone
        drone_challenge = secrets.token_bytes(16)
        
        # Send challenge
        secure_channel.send_encrypted(client_socket, "PUF_ENROLL", {"challenge": drone_challenge.hex()})
        
        # Receive response
        msg_type, msg_data = secure_channel.receive_encrypted(client_socket)
        if msg_type != "PUF_ENROLL_RESPONSE":
            raise ValueError("Expected PUF_ENROLL_RESPONSE")
        
        drone_response = bytes.fromhex(msg_data["response"])
        self.drone_puf_profiles[drone_id] = (drone_challenge, drone_response)
        
        self.logger.info(f"PUF profile enrolled for {drone_id}")
    
    def stop_server(self):
        """Stop server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()

class Drone:
    """Drone implementation"""
    
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.logger = logging.getLogger(f"Drone-{drone_id}")
        
        # Generate ECC P-192 key pair (192-bit)
        self.private_key = ec.generate_private_key(ec.SECP192R1())
        self.public_key = self.private_key.public_key()
        
        # Log key size for verification
        key_size = len(self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
        self.logger.info(f"Drone key size: {key_size} bytes")
        
        # Initialize PUF
        self.puf = PUFSimulator(drone_id)
        
        # Storage
        self.certificate = None
        self.ca_certificate = None
        
    def set_certificate(self, certificate: x509.Certificate, ca_cert: x509.Certificate):
        """Set certificates"""
        self.certificate = certificate
        self.ca_certificate = ca_cert
    
    def authenticate_with_gateway(self, gateway_host: str, gateway_port: int):
        """Authenticate with gateway"""
        try:
            # Connect to gateway
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(10)  # 10 second timeout
            client_socket.connect((gateway_host, gateway_port))
            
            # Send hello
            MessageProtocol.send_message(client_socket, "HELLO", {"drone_id": self.drone_id})
            
            # Receive response
            msg_type, msg_data = MessageProtocol.receive_message(client_socket)
            
            if msg_type == "CERT_GATEWAY":
                # Certificate authentication
                self._perform_certificate_authentication(client_socket, msg_data)
            elif msg_type == "ECDHE_PUBLIC":
                # PUF authentication (gateway started key exchange)
                self._perform_puf_authentication(client_socket, msg_type, msg_data)
            else:
                raise ValueError(f"Unexpected message: {msg_type}")
            
            client_socket.close()
            
        except Exception as e:
            self.logger.error(f"Authentication failed: {e}")
    
    def _perform_certificate_authentication(self, client_socket: socket.socket, initial_msg: dict):
        """Perform certificate authentication"""
        try:
            # Verify gateway certificate
            gateway_cert_pem = initial_msg["certificate"].encode()
            gateway_cert = x509.load_pem_x509_certificate(gateway_cert_pem)
            
            if not self._verify_certificate(gateway_cert):
                raise ValueError("Gateway certificate verification failed")
            
            # Send our certificate
            cert_pem = self.certificate.public_bytes(serialization.Encoding.PEM).decode()
            MessageProtocol.send_message(client_socket, "CERT_DRONE", {"certificate": cert_pem})
            
            # Perform key exchange
            session_key = self._perform_key_exchange(client_socket)
            secure_channel = SecureChannel(session_key)
            
            # PUF enrollment
            self._perform_puf_enrollment(client_socket, secure_channel)
            
            # Wait for success message
            msg_type, _ = secure_channel.receive_encrypted(client_socket)
            if msg_type == "AUTH_SUCCESS":
                self.logger.info("Certificate authentication successful")
            
        except Exception as e:
            self.logger.error(f"Certificate authentication failed: {e}")
    
    def _perform_puf_authentication(self, client_socket: socket.socket, msg_type: str, msg_data: dict):
        """Perform PUF authentication"""
        try:
            # Complete key exchange
            session_key = self._complete_key_exchange(client_socket, msg_data)
            secure_channel = SecureChannel(session_key)
            
            # Receive challenge
            msg_type, msg_data = secure_channel.receive_encrypted(client_socket)
            if msg_type != "PUF_CHALLENGE":
                raise ValueError("Expected PUF_CHALLENGE")
            
            # Generate response
            challenge = bytes.fromhex(msg_data["challenge"])
            response = self.puf.generate_response(challenge)
            
            # Send response
            secure_channel.send_encrypted(client_socket, "PUF_RESPONSE", {"response": response.hex()})
            
            # Wait for success
            msg_type, _ = secure_channel.receive_encrypted(client_socket)
            if msg_type == "AUTH_SUCCESS":
                self.logger.info("PUF authentication successful")
            
        except Exception as e:
            self.logger.error(f"PUF authentication failed: {e}")
    
    def _verify_certificate(self, certificate: x509.Certificate) -> bool:
        """Verify certificate"""
        try:
            # Check validity
            now = datetime.datetime.utcnow()
            if now < certificate.not_valid_before or now > certificate.not_valid_after:
                return False
            
            # Verify signature
            ca_public_key = self.ca_certificate.public_key()
            ca_public_key.verify(
                certificate.signature,
                certificate.tbs_certificate_bytes,
                ec.ECDSA(certificate.signature_hash_algorithm)
            )
            return True
            
        except:
            return False
    
    def _perform_key_exchange(self, client_socket: socket.socket) -> bytes:
        """Perform key exchange"""
        # Receive gateway's public key
        msg_type, msg_data = MessageProtocol.receive_message(client_socket)
        if msg_type != "ECDHE_PUBLIC":
            raise ValueError("Expected ECDHE_PUBLIC")
        
        return self._complete_key_exchange(client_socket, msg_data)
    
    def _complete_key_exchange(self, client_socket: socket.socket, msg_data: dict) -> bytes:
        """Complete key exchange"""
        # Generate our key pair (P-192 for smaller keys)
        private_key = ec.generate_private_key(ec.SECP192R1())
        public_key = private_key.public_key()
        
        # Load peer's public key
        peer_public_key = serialization.load_pem_public_key(msg_data["public_key"].encode())
        
        # Send our public key
        public_key_pem = public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ).decode()
        
        MessageProtocol.send_message(client_socket, "ECDHE_PUBLIC", {"public_key": public_key_pem})
        
        # Compute shared secret
        shared_key = private_key.exchange(ec.ECDH(), peer_public_key)
        
        # Derive session key
        session_key = HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=b'session_key',
        ).derive(shared_key)
        
        return session_key
    
    def _perform_puf_enrollment(self, client_socket: socket.socket, secure_channel: SecureChannel):
        """Perform PUF enrollment"""
        # Receive challenge
        msg_type, msg_data = secure_channel.receive_encrypted(client_socket)
        if msg_type != "PUF_ENROLL":
            raise ValueError("Expected PUF_ENROLL")
        
        # Generate response
        challenge = bytes.fromhex(msg_data["challenge"])
        response = self.puf.generate_response(challenge)
        
        # Send response
        secure_channel.send_encrypted(client_socket, "PUF_ENROLL_RESPONSE", {"response": response.hex()})


def main():
    """Main demonstration function"""
    print("🚁 Drone-Gateway Authentication System Demo (192-bit ECC, Keys)")
    print("=" * 70)
    
    try:
        # Initialize Certificate Authority
        ca = CertificateAuthority()
        print("✓ Certificate Authority initialized with ECC P-192")
        
        # Create Gateway
        gateway = Gateway("GW001")
        gateway_cert = ca.issue_certificate("GW001", gateway.public_key)
        gateway.set_certificate(gateway_cert, ca.ca_cert)
        print("✓ Gateway GW001 created with 192-bit ECC keys")
        
        # Create Drones with key size verification
        drone1 = Drone("DRONE001")
        drone1_cert = ca.issue_certificate("DRONE001", drone1.public_key)
        drone1.set_certificate(drone1_cert, ca.ca_cert)
        
        drone2 = Drone("DRONE002")
        drone2_cert = ca.issue_certificate("DRONE002", drone2.public_key)
        drone2.set_certificate(drone2_cert, ca.ca_cert)
        
        print("✓ Drones DRONE001 and DRONE002 created with 192-bit ECC keys")
        
        # Verify certificate sizes
        cert1_size = len(drone1_cert.public_bytes(serialization.Encoding.PEM))
        cert2_size = len(drone2_cert.public_bytes(serialization.Encoding.PEM))
        gateway_cert_size = len(gateway_cert.public_bytes(serialization.Encoding.PEM))
        
        print(f"📊 Certificate Sizes:")
        print(f"   Gateway Certificate: {gateway_cert_size} bytes")
        print(f"   Drone1 Certificate:  {cert1_size} bytes") 
        print(f"   Drone2 Certificate:  {cert2_size} bytes")
        
        # Start gateway server
        server_thread = threading.Thread(target=gateway.start_server, daemon=True)
        server_thread.start()
        
        print("✓ Gateway server started on localhost:8282")
        time.sleep(1)  # Allow server to start
        
        print("\n" + "=" * 70)
        print("🔐 FIRST AUTHENTICATION (Certificate-based)")
        print("=" * 70)
        
        # First authentication
        print("\n--- DRONE001 First Authentication ---")
        drone1.authenticate_with_gateway("localhost", 8282)
        
        print("\n--- DRONE002 First Authentication ---")
        drone2.authenticate_with_gateway("localhost", 8282)
        
        print("\n" + "=" * 70)
        print("⚡ SUBSEQUENT AUTHENTICATIONS (PUF-based)")
        print("=" * 70)
        
        time.sleep(1)  # Brief pause
        
        # Subsequent authentications
        print("\n--- DRONE001 Second Authentication (PUF) ---")
        drone1.authenticate_with_gateway("localhost", 8282)
        
        print("\n--- DRONE002 Second Authentication (PUF) ---")
        drone2.authenticate_with_gateway("localhost", 8282)
        
        print("\n--- DRONE001 Third Authentication (PUF) ---")
        drone1.authenticate_with_gateway("localhost", 8282)
        
        print("\n" + "=" * 70)
        print("📊 PERFORMANCE TEST")
        print("=" * 70)
        
        # Performance test
        test_drone = Drone("TESTDRONE")
        test_cert = ca.issue_certificate("TESTDRONE", test_drone.public_key)
        test_drone.set_certificate(test_cert, ca.ca_cert)
        
        # Time certificate auth
        start_time = time.time()
        test_drone.authenticate_with_gateway("localhost", 8282)
        cert_time = time.time() - start_time
        
        # Time PUF auth
        start_time = time.time()
        test_drone.authenticate_with_gateway("localhost", 8282)
        puf_time = time.time() - start_time
        
        print(f"\n📈 Performance Results:")
        print(f"Certificate Authentication: {cert_time:.3f} seconds")
        print(f"PUF Authentication:         {puf_time:.3f} seconds")
        if puf_time > 0:
            print(f"Speed Improvement:          {cert_time/puf_time:.1f}x faster")
        
        print("\n" + "=" * 70)
        print("🛡️  SECURITY FEATURES")
        print("=" * 70)
        
        
        # Show actual key sizes
        test_key = ec.generate_private_key(ec.SECP192R1())
        test_pub_key = test_key.public_key()
        
        pem_size = len(test_pub_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
        
        der_size = len(test_pub_key.public_bytes(
            encoding=serialization.Encoding.DER,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))
        
        raw_point = test_pub_key.public_numbers()
        raw_size = 49  # 1 byte prefix + 24 bytes x + 24 bytes y for P-192
        
        print(f"\n📐 Actual Key Sizes (P-192):")
        print(f"   PEM Format:        {pem_size} bytes")
        print(f"   DER Format:        {der_size} bytes") 
        print(f"   Raw Point:         {raw_size} bytes")
        
        print(f"\n{'='*70}")
        print("🎉 Demo completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error during demo: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean shutdown
        if 'gateway' in locals():
            gateway.stop_server()
        print("🔌 System shutdown complete")


if __name__ == "__main__":
    main()
