import smtplib
import ssl
from email.mime.text import MIMEText

class Email:
    def __init__(self, sender_email, pwd):
        self.port = 465  # For ssl
        self.smtp_server = "smtp.gmail.com"
        self.sender_email = sender_email
        self.password = pwd

    def send_email(self, email, subject, msg):
        msg = MIMEText(msg)
        msg['Subject'] = subject
        msg['From'] = self.sender_email
        msg['To'] = email
        context = ssl.create_default_context()
        with smtplib.SMTP_SSL(self.smtp_server, self.port, context=context) as server:
            server.login(self.sender_email, self.password)
            server.send_message(msg)


if __name__ == "__main__":
    Email('ravi11dec1990@gmail.com', "sdssds")