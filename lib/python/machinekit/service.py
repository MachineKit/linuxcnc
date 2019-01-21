# coding=utf-8
import avahi
import dbus
import os
import uuid
import re


class ZeroconfService(object):
    """
    A simple class to publish a network service with zeroconf using avahi.
    """

    def __init__(
        self,
        headline,
        port,
        stype="_http._tcp",
        subtype=None,
        domain="",
        host="",
        text=None,
        loopback=False,
    ):
        if text is None:
            text = []
        self.headline = headline
        self.stype = stype
        self.domain = domain
        self.host = host
        self.port = port
        self.text = text
        self.subtype = subtype
        self.loopback = loopback
        self.group = None

    def publish(self):
        bus = dbus.SystemBus()
        server = dbus.Interface(
            bus.get_object(avahi.DBUS_NAME, avahi.DBUS_PATH_SERVER),
            avahi.DBUS_INTERFACE_SERVER,
        )

        g = dbus.Interface(
            bus.get_object(avahi.DBUS_NAME, server.EntryGroupNew()),
            avahi.DBUS_INTERFACE_ENTRY_GROUP,
        )

        # insert fqdn in announcement
        fqdn = str(server.GetHostNameFqdn())
        text = [t % {'fqdn': fqdn} for t in self.text]
        headline = self.headline % {'fqdn': fqdn}

        iface = avahi.IF_UNSPEC
        if self.loopback:
            iface = 0

        g.AddService(
            iface,
            avahi.PROTO_INET,
            dbus.UInt32(0),
            headline,
            self.stype,
            self.domain,
            self.host,
            dbus.UInt16(self.port),
            text,
        )

        if self.subtype:
            g.AddServiceSubtype(
                iface,
                avahi.PROTO_INET,
                dbus.UInt32(0),
                headline,
                self.stype,
                self.domain,
                self.subtype,
            )

        g.Commit()
        self.group = g

    def unpublish(self):
        self.group.Reset()


class Service(object):
    """
    A simple class to publish a Machinekit network service using zeroconf.
    """

    # See https://gist.github.com/bgusach/a967e0587d6e01e889fd1d776c5f3729
    def __multi_replace(self, string, replacements, ignore_case=False):
        """
        Given a string and a dict, replaces occurrences of the dict keys found in the 
        string, with their corresponding values. The replacements will occur in "one pass", 
        i.e. there should be no clashes.
        :param str string: string to perform replacements on
        :param dict replacements: replacement dictionary {str_to_find: str_to_replace_with}
        :param bool ignore_case: whether to ignore case when looking for matches
        :rtype: str the replaced string
        """
        if ignore_case:
            replacements = dict((pair[0].lower(), pair[1]) for pair in sorted(replacements.iteritems()))
        rep_sorted = sorted(replacements, key=lambda s: (len(s), s), reverse=True)
        rep_escaped = [re.escape(replacement) for replacement in rep_sorted]
        pattern = re.compile("|".join(rep_escaped), re.I if ignore_case else 0)
        return pattern.sub(lambda match: replacements[match.group(0).lower() if ignore_case else match.group(0)], string) 

    def __init__(
        self,
        type_,
        svc_uuid,
        dsn,
        port,
        name=None,
        host=None,
        announce_format=None,
        headline=None,
        loopback=False,
        debug=False,
    ):
        self.dsn = dsn
        self.svc_uuid = svc_uuid
        self.type = type_
        self.port = port
        self.name = name
        self.headline = headline
        self.host = host
        self.loopback = loopback
        self.debug = debug

        self.stype = '_machinekit._tcp'
        self.subtype = '_%s._sub.%s' % (self.type, self.stype)

        if announce_format is None:
            announce_format = 'MK $SRVNAME on $HOSTNAME'

        if name is None:
            self.name = self.type.title()

        formatdict = {
            "$MKUUID" : self.svc_uuid,
            "$HOSTNAME" : self.host,
            "$SRVNAME" : self.name,
            "$SRVTYPE" : self.type
        }

        if headline is None:
            self.headline = self.__multi_replace(announce_format, formatdict, True)

        me = uuid.uuid1()
        self.status_txtrec = [
            str('dsn=' + self.dsn),
            str('uuid=' + self.svc_uuid),
            str('instance=' + str(me)),
            str('service=' + self.type),
        ]

        if self.debug:
            print(
                'service: dsname = {dsn} port = {port} txtrec = {txt} name = {name}'.format(
                    dsn=self.dsn, port=self.port, txt=self.status_txtrec, name=self.name
                )
            )

        self.statusService = ZeroconfService(
            self.headline,
            self.port,
            stype=self.stype,
            subtype=self.subtype,
            text=self.status_txtrec,
            loopback=self.loopback,
        )

    def publish(self):
        self.statusService.publish()

    def unpublish(self):
        self.statusService.unpublish()
