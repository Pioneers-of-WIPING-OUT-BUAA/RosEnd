import os

os.environ["DJANGO_SETTINGS_MODULE"] = "backend.test_settings"

import django
django.setup()

from django.test import Client
from django.test.utils import setup_databases, teardown_databases
from se.models.User import User
from se.util_ros import ROSClient


database = setup_databases(verbosity=0, interactive=False)
reactor_connection = None
try:
    user = User.objects.create(username="bridge-test", password="unused")
    client = Client(HTTP_AUTHORIZATION="Bearer " + user.token)
    for _ in range(2):
        response = client.get("/api/ros/connect")
        assert response.status_code == 200, response.content
        if reactor_connection is None:
            reactor_connection = ROSClient().client
        response = client.post("/api/user_ctrl/keyboard", {"direction": "w", "speed": 0.1}, content_type="application/json")
        assert response.status_code == 200, response.content
        response = client.get("/api/ros/free")
        assert response.status_code == 200, response.content
    print("Django -> ROSBridge -> MasterNode: two connect/control/disconnect cycles passed")
finally:
    if reactor_connection is not None:
        reactor_connection.terminate()
    teardown_databases(database, verbosity=0)
