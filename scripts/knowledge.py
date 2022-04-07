#!/usr/bin/env python

import sys
import rospy
import random

from experimental_robotics.srv import Knowledge, KnowledgeResponse

from armor_msgs.srv import ArmorDirective, ArmorDirectiveList, ArmorDirectiveListRequest
from armor_msgs.msg import _ArmorDirectiveReq


class KnowledgeManager:
    def __init__(self, client_id, reference_name):
        self.reference_name = reference_name
        self.client_id = client_id
        self.timeout = 10
        self._service_name = "/armor_interface_srv"
        self.owl_file_path = (
            "/root/ros_ws/src/experimental_robotics/knowledge/cluedo_ontology.owl"
        )
        self.iri = "http://www.emarolab.it/cluedo-ontology"
        self._handle = rospy.ServiceProxy(self._service_name, ArmorDirective)
        # self.hypo = rospy.get_param("/hints")
        self.load_ref_from_file(
            self.owl_file_path,
            self.iri,
            buffered_manipulation=True,
            reasoner="PELLET",
            buffered_reasoner=True,
            mounted=False,
        )
        rospy.Service("knowledge_base", Knowledge, self.clbkfnc)

    def clbkfnc(self, guess):
        _response = KnowledgeResponse()
        self.add_guess(guess)
        _completed = self.query("COMPLETED")
        _inconsistent = self.query("INCONSISTENT")
        if _completed != 0:
            if _inconsistent == 0:
                _response.result = "1"
                return _response
            else:
                _response.result = "3"
                return _response
        else:
            _response.result = "2"
            return _response

    def load_ref_from_file(
        self,
        owl_file_path,
        iri,
        buffered_manipulation=True,
        reasoner="PELLET",
        buffered_reasoner=True,
        mounted=False,
    ):
        """
        Loads an ontology into armor from an .owl file.
        Args:
            owl_file_path (str):
            iri (str):
            buffered_manipulation (bool): set buffered manipulations, default False
            reasoner (str): set which reasoner to use (PELLET, HERMIT, SNOROCKET, FACT), default PELLET
            buffered_reasoner (bool): set if reasoner should be buffered, default False
            mounted (bool): set if the client should be mounted on the reference, default False
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if mounted:
                res = self.call(
                    "LOAD",
                    "FILE",
                    "MOUNTED",
                    [
                        owl_file_path,
                        iri,
                        str(buffered_manipulation),
                        reasoner,
                        str(buffered_reasoner),
                    ],
                )
            else:
                res = self.call(
                    "LOAD",
                    "FILE",
                    "",
                    [
                        owl_file_path,
                        iri,
                        str(buffered_manipulation),
                        reasoner,
                        str(buffered_reasoner),
                    ],
                )

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon reference {0} from {1}".format(
                    self._client.reference_name, self._client.client_id
                )
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def call(self, command, first_spec, second_spec, args_list):
        req = self.request(command, first_spec, second_spec, args_list)
        rospy.wait_for_service(self._service_name, self.timeout)
        res = self._handle(req).armor_response
        return res

    def add_guess(self, guess):
        self.guess = guess.guess.split(" ")
        self.attribute = []

        where = ["Kitchen", "Bedroom", "Bathroom", "Library", "Garage", "Living Room"]
        who = ["Joseph", "Mark", "Mabel", "Romero", "Almate", "Bruno"]
        what = ["Broom", "Stick", "Knife", "Bucket", "Gun", "Tissue", "Slippers"]

        for i in self.guess:
            if i in where:
                self.attribute.append("where")
            if i in who:
                self.attribute.append("who")
            if i in what:
                self.attribute.append("what")

        for i, a in enumerate(self.guess):
            k = 0
            if self.attribute[i] == "where":
                self.ind_cls("PLACE", k)
                self.disjoint_reason("PLACE")
                self.obj_prop("PLACE", k, i)
                self.disjoint_reason("PLACE")

            if self.attribute[i] == "who":
                self.ind_cls("PERSON", k)
                self.disjoint_reason("PERSON")
                self.obj_prop("PERSON", k, i)
                self.disjoint_reason("PERSON")

            if self.attribute[i] == "what":
                self.ind_cls("WEAPON", k)
                self.disjoint_reason("WEAPON")
                self.obj_prop("WEAPON", k, i)
                self.disjoint_reason("WEAPON")
        k += 1

    def request(self, command, pri_spec, sec_spec, args):
        req = _ArmorDirectiveReq.ArmorDirectiveReq()
        req.client_name = self.client_id
        req.reference_name = self.reference_name
        req.command = command
        req.primary_command_spec = pri_spec
        req.secondary_command_spec = sec_spec
        req.args = args
        return req

    def disjoint_reason(self, cls):
        req = self.request("DISJOINT", "IND", "CLASS", cls)
        rospy.wait_for_service(self._service_name, self.timeout)
        resp = self._handle(req).armor_response

        req = self.request("REASON", "", "", [])
        rospy.wait_for_service(self._service_name, self.timeout)
        resp = self._handle(req).armor_response
        return resp

    def ind_cls(self, cls, i):
        request = self.request("ADD", "IND", "CLASS", [self.guess[i], cls])
        rospy.wait_for_service(self._service_name, self.timeout)
        response = self._handle(request).armor_response
        return response

    def obj_prop(self, cls, k, i):
        request = self.request(
            "ADD", "OBJECTPROP", "IND", [self.attribute[i], f"HP{k}", self.guess[i]]
        )
        rospy.wait_for_service(self._service_name, self.timeout)
        response = self._handle(request).armor_response
        return response

    def query(self, what_to_check):
        req = self.request("QUERY", "IND", "CLASS", [what_to_check])
        rospy.wait_for_service(self._service_name, self.timeout)
        resp = self._handle(req).armor_response
        return resp


def main():
    rospy.init_node("knowledge_base", anonymous=False)
    print("The service is about to start")
    KnowledgeManager(rospy.get_name(), "cluedo_game")
    rospy.spin()


if __name__ == "__main__":
    main()
